// platelog -- force plate + carriage encoder acquisition, the hot loop only.
// The live UDP feed is decimated to ~100 Hz by default; "DIV 1" from the
// dashboard switches it to every sample (packed, for one-shot capture).
//
// Reads an ADS131M08 (4 load cells on ch0..3) over spidev and a quadrature
// encoder through the kernel rotary-encoder input device, and writes one CSV
// row per ADC conversion. Everything slow -- calibration files, tare storage,
// plotting -- lives in leglog.py, which drives this binary.
//
// Why C++: the ADS131M08 does not buffer. If a conversion completes before the
// previous one was read, the previous one is gone, silently. Reading it on
// time at 1-4 kSPS while also draining encoder events and writing a file is
// exactly the kind of loop a garbage collector or GIL hiccup breaks, and a
// missed sample in an impact trace is the sample you wanted.
//
// Frame protocol (24-bit words, 16-bit values left-aligned):
//   send [cmd][pad]x9      receive [status][ch0..ch7][crc]
// The reply to a register command lands in the *next* frame (offset 1), or
// later if a previous process died mid-frame; sync() measures the real offset
// against MODE's known post-reset value 0x0510 rather than assuming it.
//
// Build:  g++ -O2 -std=c++17 -o platelog platelog.cpp
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <getopt.h>
#include <linux/input.h>
#include <linux/spi/spidev.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int NCH = 8;
constexpr int WORDS = NCH + 2;
constexpr int FLEN = WORDS * 3;
constexpr int CELLS = 4;
constexpr double G0 = 9.80665;

constexpr uint16_t CMD_NULL = 0x0000, CMD_RESET = 0x0011;
constexpr uint8_t REG_STATUS = 0x01, REG_MODE = 0x02, REG_CLOCK = 0x03,
                  REG_GAIN1 = 0x04, REG_GAIN2 = 0x05;

volatile std::sig_atomic_t g_stop = 0, g_toggle_rec = 0, g_retare = 0, g_zeroz = 0;
void on_sigint(int) { g_stop = 1; }
void on_usr1(int) { g_toggle_rec = 1; }
void on_usr2(int) { g_retare = 1; }

double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

uint16_t crc16_ccitt(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t>(d[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

int gain_code(int g) {
    switch (g) { case 1: return 0; case 2: return 1; case 4: return 2; case 8: return 3;
                 case 16: return 4; case 32: return 5; case 64: return 6; case 128: return 7; }
    return -1;
}
int osr_code(int o) {
    switch (o) { case 128: return 0; case 256: return 1; case 512: return 2; case 1024: return 3;
                 case 2048: return 4; case 4096: return 5; case 8192: return 6; case 16384: return 7; }
    return -1;
}

// ----------------------------------------------------------------- ADS131M08
class Ads131 {
public:
    bool open(const char* dev, uint32_t speed) {
        fd_ = ::open(dev, O_RDWR);
        if (fd_ < 0) { perror(dev); return false; }
        uint8_t mode = SPI_MODE_1, bits = 8;
        speed_ = speed;
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
            perror("spi ioctl"); return false;
        }
        return true;
    }

    bool xfer(uint16_t cmd, uint16_t payload = 0, bool has_payload = false) {
        std::memset(tx_, 0, FLEN);
        tx_[0] = cmd >> 8; tx_[1] = cmd & 0xFF;
        if (has_payload) { tx_[3] = payload >> 8; tx_[4] = payload & 0xFF; }
        spi_ioc_transfer tr{};
        tr.tx_buf = reinterpret_cast<uint64_t>(tx_);
        tr.rx_buf = reinterpret_cast<uint64_t>(rx_);
        tr.len = FLEN; tr.speed_hz = speed_; tr.bits_per_word = 8;
        return ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) >= 1;
    }

    uint32_t word(int i) const {
        return (uint32_t(rx_[3 * i]) << 16) | (uint32_t(rx_[3 * i + 1]) << 8) | rx_[3 * i + 2];
    }
    uint16_t status() const { return word(0) >> 8; }
    bool crc_ok() const { return crc16_ccitt(rx_, FLEN - 3) == (word(WORDS - 1) >> 8); }
    int32_t ch(int i) const {
        uint32_t w = word(1 + i);
        return (w & 0x800000) ? int32_t(w) - (1 << 24) : int32_t(w);
    }

    int read_reg(uint8_t addr) {
        xfer(CMD_NULL); xfer(CMD_NULL);                      // park the pipeline
        xfer(0xA000 | (uint16_t(addr & 0x3F) << 7));
        for (int k = 0; k < resp_off_; ++k) xfer(CMD_NULL);
        return status();
    }
    bool write_reg(uint8_t addr, uint16_t val) {
        for (int attempt = 0; attempt < 3; ++attempt) {
            xfer(CMD_NULL); xfer(CMD_NULL);
            xfer(0x6000 | (uint16_t(addr & 0x3F) << 7), val, true);
            for (int k = 0; k < resp_off_; ++k) xfer(CMD_NULL);
            if (read_reg(addr) == val) return true;
            if (attempt == 0) sync();
        }
        return false;
    }

    // Reset, then find how many frames behind the command a response lands.
    bool sync() {
        for (int t = 0; t < 3; ++t) {
            xfer(CMD_RESET);
            usleep(10000);
            xfer(CMD_NULL); xfer(CMD_NULL);
            xfer(0xA000 | (uint16_t(REG_MODE) << 7));
            for (int off = 1; off <= 4; ++off) {
                xfer(CMD_NULL);
                if (status() == 0x0510) { resp_off_ = off; return true; }
            }
        }
        return false;
    }

    bool configure(int gain, int osr, std::string& err) {
        int g = gain_code(gain), o = osr_code(osr);
        if (g < 0) { err = "bad gain"; return false; }
        if (o < 0) { err = "bad osr"; return false; }
        if (!sync()) { err = "ADS131 not answering (MODE never read 0x0510) -- is CLKIN running on GPIO4?"; return false; }
        uint16_t gg = uint16_t(g | (g << 4) | (g << 8) | (g << 12));
        gg_ = gg;
        if (!write_reg(REG_GAIN1, gg)) { err = "GAIN1 write failed"; return false; }
        if (!write_reg(REG_GAIN2, gg)) { err = "GAIN2 write failed"; return false; }
        uint16_t clk = uint16_t(0xFF00 | (o << 2) | 0b10);      // all ch on, high-res
        clk_ = clk;
        if (!write_reg(REG_CLOCK, clk)) { err = "CLOCK write failed"; return false; }
        return true;
    }

    // Read GAIN1, GAIN2 and CLOCK back and rewrite any that no longer hold what
    // configure() put there.  Motor EMI has twice changed them mid-run without a
    // restart (OSR -> 4 kSPS on 2026-09-26, PGA gain 64 -> 1 on 2026-10-03), and
    // nothing in the data stream says so.  Returns a bitmask of the registers that
    // were wrong (1 GAIN1, 2 GAIN2, 4 CLOCK; 8 = a rewrite failed); got[] = what
    // was read.  Costs ~15 SPI frames, i.e. 1-2 samples.
    int verify_fix(uint16_t got[3]) {
        const uint8_t addr[3] = {REG_GAIN1, REG_GAIN2, REG_CLOCK};
        const uint16_t want[3] = {gg_, gg_, clk_};
        int bad = 0;
        for (int k = 0; k < 3; ++k) {
            got[k] = uint16_t(read_reg(addr[k]));
            if (got[k] != want[k]) {
                bad |= 1 << k;
                if (!write_reg(addr[k], want[k])) bad |= 8;
            }
        }
        return bad;
    }
    uint16_t want_gain() const { return gg_; }
    uint16_t want_clock() const { return clk_; }

    // True when this frame carries a conversion not yet read (DRDY bits for ch0..3).
    bool fresh() const { return (status() & 0x0F) == 0x0F; }

private:
    int fd_ = -1;
    uint32_t speed_ = 2000000;
    int resp_off_ = 1;
    uint16_t gg_ = 0, clk_ = 0;
    uint8_t tx_[FLEN]{}, rx_[FLEN]{};
};

// -------------------------------------------------------------------- encoder
class Encoder {
public:
    bool open() {
        // by-path link is stable; fall back to scanning names
        char buf[256];
        for (int i = 0; i < 32; ++i) {
            std::snprintf(buf, sizeof buf, "/sys/class/input/event%d/device/name", i);
            FILE* f = std::fopen(buf, "r");
            if (!f) continue;
            char name[64] = {0};
            if (std::fgets(name, sizeof name, f) && std::strncmp(name, "rotary@", 7) == 0) {
                std::fclose(f);
                std::snprintf(buf, sizeof buf, "/dev/input/event%d", i);
                fd_ = ::open(buf, O_RDONLY | O_NONBLOCK);
                if (fd_ >= 0) { path_ = buf; return true; }
                perror(buf);
                return false;
            }
            std::fclose(f);
        }
        return false;
    }
    void poll() {
        if (fd_ < 0) return;
        input_event ev[256];
        for (;;) {
            ssize_t n = ::read(fd_, ev, sizeof ev);
            if (n <= 0) break;
            size_t cnt = size_t(n) / sizeof(input_event);
            for (size_t i = 0; i < cnt; ++i)
                if (ev[i].type == EV_REL && ev[i].code == REL_X) count += ev[i].value;
            if (cnt < 256) break;
        }
    }
    bool ok() const { return fd_ >= 0; }
    const std::string& path() const { return path_; }
    long count = 0;
private:
    int fd_ = -1;
    std::string path_;
};

// --------------------------------------------------------------- file writer
// The acquisition loop appends text to an in-memory buffer; this thread owns
// the file. A 1 MB stdio flush to the SD card was measured at ~2 ms, which is
// eight lost conversions at 4 kSPS -- and it hit like clockwork every 2.8 s.
class Writer {
public:
    bool open(const std::string& path) {
        f_ = std::fopen(path.c_str(), "w");
        if (!f_) { perror(path.c_str()); return false; }
        th_ = std::thread([this] { run(); });
        return true;
    }
    void append(const char* s, size_t n) {
        std::lock_guard<std::mutex> lk(m_);
        buf_.append(s, n);
        if (buf_.size() >= 64 * 1024) cv_.notify_one();
    }
    void close() {
        { std::lock_guard<std::mutex> lk(m_); done_ = true; }
        cv_.notify_one();
        if (th_.joinable()) th_.join();
        if (f_) std::fclose(f_);
        f_ = nullptr;
    }
    bool ok() const { return f_ != nullptr; }
private:
    void run() {
        std::string local;
        for (;;) {
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait_for(lk, std::chrono::milliseconds(200),
                             [this] { return done_ || buf_.size() >= 64 * 1024; });
                local.swap(buf_);
                if (local.empty() && done_) return;
            }
            if (!local.empty()) { std::fwrite(local.data(), 1, local.size(), f_); local.clear(); }
        }
    }
    FILE* f_ = nullptr;
    std::thread th_;
    std::mutex m_;
    std::condition_variable cv_;
    std::string buf_;
    bool done_ = false;
};

// ------------------------------------------------------------------ UDP feed
// Sends run on their own thread: a send() into the WiFi driver was seen to
// stall the caller for a couple of milliseconds now and then, which at 1 kSPS
// is a lost conversion. The acquisition loop only ever touches a mutex here.
class UdpFeed {
public:
    bool open(const std::string& hostport) {
        auto c = hostport.rfind(':');
        if (c == std::string::npos) return false;
        std::string host = hostport.substr(0, c), port = hostport.substr(c + 1);
        addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET; hints.ai_socktype = SOCK_DGRAM;
        if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) return false;
        fd_ = socket(res->ai_family, res->ai_socktype, 0);
        if (fd_ >= 0 && connect(fd_, res->ai_addr, res->ai_addrlen) != 0) { ::close(fd_); fd_ = -1; }
        freeaddrinfo(res);
        if (fd_ >= 0) th_ = std::thread([this] { run(); });
        return fd_ >= 0;
    }
    void send(const char* s, size_t n) {
        if (fd_ < 0) return;
        std::lock_guard<std::mutex> lk(m_);
        if (q_.size() > 50) q_.pop_front();          // receiver is gone: don't pile up
        q_.emplace_back(s, n);
        cv_.notify_one();
    }
    // Commands from the dashboard arrive on this same connected socket.
    int recv(char* buf, size_t n) {
        if (fd_ < 0) return 0;
        ssize_t r = ::recv(fd_, buf, n - 1, MSG_DONTWAIT);
        if (r <= 0) return 0;
        buf[r] = 0;
        return int(r);
    }
    void close() {
        { std::lock_guard<std::mutex> lk(m_); done_ = true; }
        cv_.notify_one();
        if (th_.joinable()) th_.join();
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }
    bool ok() const { return fd_ >= 0; }
private:
    void run() {
        for (;;) {
            std::string d;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [this] { return done_ || !q_.empty(); });
                if (q_.empty() && done_) return;
                d = std::move(q_.front()); q_.pop_front();
            }
            ::send(fd_, d.data(), d.size(), MSG_DONTWAIT);   // errors (no listener) are fine
        }
    }
    int fd_ = -1;
    std::thread th_;
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<std::string> q_;
    bool done_ = false;
};

// ----------------------------------------------------------------------- args
struct Args {
    int gain = 64, osr = 4096;
    double seconds = 0;              // 0 = until ctrl-C
    std::string out;                 // empty = no file (unless toggled)
    std::string out_dir;             // SIGUSR1 starts/stops a timestamped file here
    std::string udp;                 // host:port for the decimated live feed
    int udp_div = 0;                 // send every Nth sample (0 = auto ~100 Hz)
    int udp_batch = 0;               // samples packed per datagram (0 = auto)
    double zeros[CELLS] = {0, 0, 0, 0};
    double newton[CELLS] = {0, 0, 0, 0};   // N per count, per cell
    double m_per_count = 0;
    double tare_seconds = 0;
    std::string tare_file;           // read zeros from here if no --zeros; rewrite on every tare
    bool tare_only = false;
    double status = 0.5;             // stderr status interval, 0 = off
    bool status_newline = false;
    bool no_crc = false;
    bool no_encoder = false;
    double reg_check = 1.0;          // s between ADC register read-backs, 0 = never
    uint32_t spi_speed = 2000000;
    std::string spidev = "/dev/spidev0.0";
    std::string note;
};

bool parse4(const char* s, double* out) {
    return std::sscanf(s, "%lf,%lf,%lf,%lf", &out[0], &out[1], &out[2], &out[3]) == 4;
}

void usage() {
    std::fprintf(stderr,
        "platelog [options]\n"
        "  --gain G          PGA gain 1..128 (64)\n"
        "  --osr N           oversampling 128..16384; 4096 = 1 kSPS, 1024 = 4 kSPS (4096)\n"
        "  --seconds S       stop after S seconds (0 = run until ctrl-C)\n"
        "  --out FILE        CSV output (none = just status)\n"
        "  --out-dir DIR     SIGUSR1 toggles recording to DIR/plate-<timestamp>.csv\n"
        "  --udp HOST:PORT   send a decimated live feed (text datagrams)\n"
        "  --udp-div N       send one sample per N (default: ~100 Hz; 1 = full rate)\n"
        "  --udp-batch N     pack N samples into one datagram (default: 1, or 15 when div=1)\n"
        "  (SIGUSR2 re-tares over 2 s -- plate must be empty)\n"
        "  --zeros a,b,c,d   tare counts per cell\n"
        "  --newton a,b,c,d  newtons per count per cell\n"
        "  --m-per-count X   encoder metres per count (signed)\n"
        "  --tare-seconds S  measure zeros for S seconds before logging\n"
        "  --tare-file F     JSON: load zeros from F unless --zeros/--tare-seconds; rewrite on tare\n"
        "  UDP commands from the feed peer: REC (toggle), TARE, ZEROZ, PING, DIV <n>\n"
        "    DIV 1 switches the live feed to every sample (batched); DIV 10 back to ~100 Hz.\n"
        "  --tare-only       measure zeros, print JSON, exit\n"
        "  --status S        status line interval in seconds (0.5; 0 = off)\n"
        "  --status-newline  one status line per update instead of \\r\n"
        "  --no-crc          skip per-frame CRC (not needed even at 4 kSPS)\n"
        "  --no-encoder      do not open the encoder device\n"
        "  --reg-check S     read back GAIN/CLOCK every S s and rewrite if lost (default 1, 0 = off)\n"
        "  --note TEXT       free text stored in the CSV header\n");
}

bool read_tare_file(const std::string& path, double* zeros) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;
    char buf[1024] = {0};
    size_t n = std::fread(buf, 1, sizeof buf - 1, f);
    std::fclose(f);
    buf[n] = 0;
    const char* p = std::strstr(buf, "\"zeros\"");
    if (!p) return false;
    p = std::strchr(p, '[');
    return p && std::sscanf(p, "[%lf,%lf,%lf,%lf", &zeros[0], &zeros[1], &zeros[2], &zeros[3]) == 4;
}

void write_tare_file(const std::string& path, const double* zeros, const double* sd, long n, int gain, int osr) {
    if (path.empty()) return;
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) { perror(path.c_str()); return; }
    std::fprintf(f, "{\"zeros\": [%.2f, %.2f, %.2f, %.2f], \"sd\": [%.1f, %.1f, %.1f, %.1f], "
                    "\"n\": %ld, \"gain\": %d, \"osr\": %d, \"time\": %.3f}\n",
                 zeros[0], zeros[1], zeros[2], zeros[3], sd[0], sd[1], sd[2], sd[3], n, gain, osr,
                 double(std::time(nullptr)));
    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    Args a;
    static option opts[] = {
        {"gain", 1, nullptr, 'g'}, {"osr", 1, nullptr, 'o'}, {"seconds", 1, nullptr, 's'},
        {"out", 1, nullptr, 'f'}, {"out-dir", 1, nullptr, 'D'}, {"udp", 1, nullptr, 'u'},
        {"udp-div", 1, nullptr, 'v'}, {"udp-batch", 1, nullptr, 'B'},
        {"zeros", 1, nullptr, 'z'}, {"newton", 1, nullptr, 'n'},
        {"m-per-count", 1, nullptr, 'm'}, {"tare-seconds", 1, nullptr, 't'},
        {"tare-only", 0, nullptr, 'T'}, {"tare-file", 1, nullptr, 'F'}, {"status", 1, nullptr, 'S'},
        {"status-newline", 0, nullptr, 'L'}, {"no-crc", 0, nullptr, 'C'},
        {"no-encoder", 0, nullptr, 'E'}, {"spi-speed", 1, nullptr, 'p'},
        {"spidev", 1, nullptr, 'd'}, {"note", 1, nullptr, 'N'}, {"help", 0, nullptr, 'h'},
        {"reg-check", 1, nullptr, 'K'},
        {nullptr, 0, nullptr, 0}};
    int c;
    while ((c = getopt_long(argc, argv, "h", opts, nullptr)) != -1) {
        switch (c) {
            case 'g': a.gain = std::atoi(optarg); break;
            case 'o': a.osr = std::atoi(optarg); break;
            case 's': a.seconds = std::atof(optarg); break;
            case 'f': a.out = optarg; break;
            case 'D': a.out_dir = optarg; break;
            case 'u': a.udp = optarg; break;
            case 'v': a.udp_div = std::atoi(optarg); break;
            case 'B': a.udp_batch = std::atoi(optarg); break;
            case 'z': if (!parse4(optarg, a.zeros)) { usage(); return 2; } break;
            case 'n': if (!parse4(optarg, a.newton)) { usage(); return 2; } break;
            case 'm': a.m_per_count = std::atof(optarg); break;
            case 't': a.tare_seconds = std::atof(optarg); break;
            case 'T': a.tare_only = true; break;
            case 'F': a.tare_file = optarg; break;
            case 'S': a.status = std::atof(optarg); break;
            case 'L': a.status_newline = true; break;
            case 'C': a.no_crc = true; break;
            case 'E': a.no_encoder = true; break;
            case 'p': a.spi_speed = uint32_t(std::atol(optarg)); break;
            case 'd': a.spidev = optarg; break;
            case 'N': a.note = optarg; break;
            case 'K': a.reg_check = std::atof(optarg); break;
            default: usage(); return 2;
        }
    }
    std::signal(SIGINT, on_sigint);
    std::signal(SIGTERM, on_sigint);
    std::signal(SIGUSR1, on_usr1);
    std::signal(SIGUSR2, on_usr2);

    bool have_zeros = a.zeros[0] != 0 || a.zeros[1] != 0 || a.zeros[2] != 0 || a.zeros[3] != 0;
    if (!have_zeros && a.tare_seconds <= 0 && !a.tare_only && !a.tare_file.empty()) {
        if (read_tare_file(a.tare_file, a.zeros))
            std::fprintf(stderr, "zeros from %s: %.0f %.0f %.0f %.0f\n", a.tare_file.c_str(),
                         a.zeros[0], a.zeros[1], a.zeros[2], a.zeros[3]);
        else { std::fprintf(stderr, "no usable %s -- taring for 2 s (plate must be empty)\n", a.tare_file.c_str());
               a.tare_seconds = 2; }
    }

    // Real-time priority if the system allows it; otherwise carry on.
    sched_param sp{}; sp.sched_priority = 40;
    bool rt = sched_setscheduler(0, SCHED_FIFO, &sp) == 0;
    bool locked = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;    // no page faults mid-loop
    if (!rt) std::fprintf(stderr, "note: no realtime priority (rtprio limit is %s) -- "
                                  "expect a few lost samples per minute\n",
                          locked ? "set but SCHED_FIFO refused" : "0");

    Ads131 adc;
    if (!adc.open(a.spidev.c_str(), a.spi_speed)) return 1;
    std::string err;
    if (!adc.configure(a.gain, a.osr, err)) { std::fprintf(stderr, "error: %s\n", err.c_str()); return 1; }
    for (int i = 0; i < 60; ++i) {            // let the sinc filter refill
        while (!(adc.xfer(CMD_NULL) && adc.fresh())) {}
    }

    Encoder enc;
    bool have_enc = !a.no_encoder && enc.open();
    if (!a.no_encoder && !have_enc)
        std::fprintf(stderr, "warning: no rotary@ input device -- encoder column will be 0 "
                             "(sudo dtoverlay rotary-encoder pin_a=5 pin_b=6 relative_axis=1)\n");

    const double period = 2.0 * a.osr / 8.192e6;   // nominal, refined from measurement below
    const bool spin = period < 0.0006;              // below ~1.6 kSPS, don't trust nanosleep

    // ---- tare -----------------------------------------------------------------
    if (a.tare_seconds > 0 || a.tare_only) {
        double secs = a.tare_seconds > 0 ? a.tare_seconds : 3.0;
        double sum[CELLS] = {0}, sq[CELLS] = {0};
        long n = 0;
        double t0 = now_s();
        while (now_s() - t0 < secs && !g_stop) {
            if (!adc.xfer(CMD_NULL) || !adc.fresh()) continue;
            if (!a.no_crc && !adc.crc_ok()) continue;
            for (int j = 0; j < CELLS; ++j) { double v = adc.ch(j); sum[j] += v; sq[j] += v * v; }
            ++n;
        }
        if (n < 10) { std::fprintf(stderr, "error: tare got only %ld samples\n", n); return 1; }
        double sd[CELLS];
        for (int j = 0; j < CELLS; ++j) {
            a.zeros[j] = sum[j] / n;
            sd[j] = std::sqrt(std::max(0.0, sq[j] / n - a.zeros[j] * a.zeros[j]));
        }
        if (a.tare_only) {
            std::printf("{\"zeros\": [%.2f, %.2f, %.2f, %.2f], \"sd\": [%.1f, %.1f, %.1f, %.1f], "
                        "\"n\": %ld, \"gain\": %d, \"osr\": %d}\n",
                        a.zeros[0], a.zeros[1], a.zeros[2], a.zeros[3], sd[0], sd[1], sd[2], sd[3],
                        n, a.gain, a.osr);
            return 0;
        }
        std::fprintf(stderr, "tared: %.0f %.0f %.0f %.0f  (n=%ld)\n",
                     a.zeros[0], a.zeros[1], a.zeros[2], a.zeros[3], n);
        write_tare_file(a.tare_file, a.zeros, sd, n, a.gain, a.osr);
    }

    // ---- output -----------------------------------------------------------------
    Writer out;
    char line[512];
    auto emit = [&](const char* fmt, auto... args) {
        int n = std::snprintf(line, sizeof line, fmt, args...);
        if (n > 0) out.append(line, size_t(n));
    };
    std::string rec_path;
    long rec_i0 = 0;
    double rec_t0 = 0;
    auto start_rec = [&](const std::string& path) {
        if (!out.open(path)) return false;
        rec_path = path;
        char ts[32];
        std::time_t tt = std::time(nullptr);
        std::strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", std::localtime(&tt));
        emit("# platelog v1\n# start %s\n# gain %d osr %d nominal_sps %.1f crc %s rt %s\n",
             ts, a.gain, a.osr, 1.0 / period, a.no_crc ? "off" : "on", rt ? "fifo" : "normal");
        emit("# zeros %.2f %.2f %.2f %.2f\n", a.zeros[0], a.zeros[1], a.zeros[2], a.zeros[3]);
        emit("# newton_per_count %.9g %.9g %.9g %.9g\n", a.newton[0], a.newton[1], a.newton[2], a.newton[3]);
        emit("# encoder %s m_per_count %.9g\n", have_enc ? enc.path().c_str() : "none", a.m_per_count);
        if (!a.note.empty()) emit("# note %s\n", a.note.c_str());
        emit("%s", "t_s,i,fz_N,enc,z_m,f0_N,f1_N,f2_N,f3_N,raw0,raw1,raw2,raw3\n");
        std::fprintf(stderr, "\nrecording -> %s\n", path.c_str());
        return true;
    };
    auto stop_rec = [&](long i, double t, long gaps, long crc_err, double fmin, double fmax) {
        if (!out.ok()) return;
        double el = t - rec_t0;
        long n = i - rec_i0;
        emit("# end samples %ld seconds %.3f sps %.1f expected_from_clock %ld gaps %ld crc_errors %ld fz_min %.3f fz_max %.3f\n",
             n, el, el > 0 ? n / el : 0.0, long(std::lround(el / period)) + 1, gaps, crc_err, fmin, fmax);
        out.close();
        std::fprintf(stderr, "\nclosed %s: %ld samples in %.2f s\n", rec_path.c_str(), n, el);
    };

    UdpFeed feed;
    if (!a.udp.empty() && !feed.open(a.udp)) std::fprintf(stderr, "warning: cannot open UDP feed %s\n", a.udp.c_str());
    const int udp_div_slow = a.udp_div > 0 ? a.udp_div : std::max(1, int(std::lround(0.01 / period)));
    int udp_div = udp_div_slow;
    // One datagram per sample is fine at 100 Hz, but at the full 1-4 kSPS it is
    // thousands of sendto()s a second, so those get packed.  Stay under a
    // typical 1500 B MTU: a fragmented datagram is lost whole if any fragment is.
    constexpr size_t MAX_DGRAM = 1400;
    auto auto_batch = [&](int div) { return a.udp_batch > 0 ? a.udp_batch : (div == 1 ? 15 : 1); };
    int udp_batch = auto_batch(udp_div);
    std::string dgram;
    dgram.reserve(2 * MAX_DGRAM);          // reserved once; .clear() keeps the capacity
    int dgram_n = 0;
    auto dgram_flush = [&] {
        if (dgram.empty()) return;
        feed.send(dgram.data(), dgram.size());
        dgram.clear();
        dgram_n = 0;
    };

    // ---- main loop --------------------------------------------------------------
    if (have_enc) enc.poll();
    long enc0 = enc.count;                     // z is relative to where we started
    long i = 0, gaps = 0, crc_err = 0, reg_fixes = 0;
    double t_regchk = now_s();
    // Test hook for the read-back: PLATELOG_TEST_CORRUPT=S sets the PGA gain to 1
    // once, S seconds in -- what EMI did on 2026-10-03 -- so the repair can be seen.
    const char* tc_env = std::getenv("PLATELOG_TEST_CORRUPT");
    double test_corrupt_at = tc_env ? std::atof(tc_env) : -1;
    double t0 = now_s(), t_prev = -1, t_status = t0, t_rate = t0;
    long i_rate = 0;
    double fz = 0, z = 0, fz_min = 1e9, fz_max = -1e9;     // whole run
    double rec_min = 1e9, rec_max = -1e9;                   // current file
    double bin_min = 1e9, bin_max = -1e9;                   // current UDP bin
    long bin_n = 0;
    double tare_sum[CELLS] = {0}, tare_sq[CELLS] = {0}; long tare_n = 0, tare_want = 0;
    timespec nap{0, 100000};                   // 100 us
    const double gap_thresh = 1.95 * period;   // the ADC overwrites only after ~2 periods

    if (!a.out.empty() && !start_rec(a.out)) return 1;

    while (!g_stop) {
        if (!adc.xfer(CMD_NULL)) { std::fprintf(stderr, "spi transfer failed\n"); break; }
        if (!adc.fresh()) { if (!spin) nanosleep(&nap, nullptr); continue; }
        double t = now_s() - t0;
        if (a.seconds > 0 && t >= a.seconds) break;
        if (!a.no_crc && !adc.crc_ok()) { ++crc_err; continue; }
        if (have_enc) enc.poll();

        if (g_toggle_rec) {
            g_toggle_rec = 0;
            if (out.ok()) stop_rec(i, t, gaps, crc_err, rec_min, rec_max);
            else if (!a.out_dir.empty()) {
                char ts[32];
                std::time_t tt = std::time(nullptr);
                std::strftime(ts, sizeof ts, "%Y%m%d-%H%M%S", std::localtime(&tt));
                if (start_rec(a.out_dir + "/plate-" + ts + ".csv")) {
                    rec_i0 = i; rec_t0 = t; rec_min = 1e9; rec_max = -1e9;
                }
            }
        }
        if (g_retare) {
            g_retare = 0;
            tare_want = long(std::lround(2.0 / period));
            tare_n = 0;
            for (double& s : tare_sum) s = 0;
            for (double& s : tare_sq) s = 0;
            std::fprintf(stderr, "\nre-taring over 2 s ...\n");
        }
        if (tare_want > 0) {
            for (int j = 0; j < CELLS; ++j) { double v = adc.ch(j); tare_sum[j] += v; tare_sq[j] += v * v; }
            if (++tare_n >= tare_want) {
                double sd[CELLS];
                for (int j = 0; j < CELLS; ++j) {
                    a.zeros[j] = tare_sum[j] / tare_n;
                    sd[j] = std::sqrt(std::max(0.0, tare_sq[j] / tare_n - a.zeros[j] * a.zeros[j]));
                }
                tare_want = 0;
                std::fprintf(stderr, "tared: %.0f %.0f %.0f %.0f\n", a.zeros[0], a.zeros[1], a.zeros[2], a.zeros[3]);
                write_tare_file(a.tare_file, a.zeros, sd, tare_n, a.gain, a.osr);
                if (out.ok()) emit("# retare %.6f %.2f %.2f %.2f %.2f\n", t, a.zeros[0], a.zeros[1], a.zeros[2], a.zeros[3]);
            }
        }
        if (g_zeroz) {
            g_zeroz = 0;
            enc0 = enc.count;
            if (out.ok()) emit("# zeroz %.6f enc %ld\n", t, enc.count);
        }
        // Commands from the dashboard, on the feed socket. Checked every
        // sample; a non-blocking recv on an empty socket is a few microseconds.
        if (feed.ok()) {
            char cbuf[64];
            if (feed.recv(cbuf, sizeof cbuf)) {
                if (!std::strncmp(cbuf, "REC", 3)) g_toggle_rec = 1;
                else if (!std::strncmp(cbuf, "TARE", 4)) g_retare = 1;
                else if (!std::strncmp(cbuf, "ZEROZ", 5)) g_zeroz = 1;
                else if (!std::strncmp(cbuf, "DIV", 3)) {
                    // "DIV 1" = stream every sample, "DIV 0" = back to the default
                    // ~100 Hz.  The one-shot mode of the dashboard needs full rate.
                    int nd = std::atoi(cbuf + 3);
                    if (nd == 0) nd = udp_div_slow;
                    if (nd >= 1 && nd <= 100000 && nd != udp_div) {
                        dgram_flush();
                        udp_div = nd;
                        udp_batch = auto_batch(udp_div);
                        bin_n = 0; bin_min = 1e9; bin_max = -1e9;
                        std::fprintf(stderr, "\nudp feed: div %d, %d sample(s)/datagram (%.0f dg/s)\n",
                                     udp_div, udp_batch, 1.0 / (period * udp_div * udp_batch));
                    }
                }
                else if (!std::strncmp(cbuf, "PING", 4)) {
                    char pb[128];
                    int pn = std::snprintf(pb, sizeof pb, "PONG,%d,%d,%d,%d,%.1f,%ld\n",
                                           udp_div, udp_batch, a.osr, a.gain, 1.0 / period, reg_fixes);
                    if (pn > 0) feed.send(pb, size_t(pn));
                }
            }
        }

        if (t_prev >= 0) {
            double dt = t - t_prev;
            if (dt > gap_thresh) gaps += std::max(1L, long(std::lround(dt / period)) - 1);
        }
        t_prev = t;

        double f[CELLS];
        fz = 0;
        for (int j = 0; j < CELLS; ++j) { f[j] = a.newton[j] * (adc.ch(j) - a.zeros[j]); fz += f[j]; }
        z = (enc.count - enc0) * a.m_per_count;
        fz_min = std::min(fz_min, fz); fz_max = std::max(fz_max, fz);
        rec_min = std::min(rec_min, fz); rec_max = std::max(rec_max, fz);
        bin_min = std::min(bin_min, fz); bin_max = std::max(bin_max, fz);

        if (out.ok())
            emit("%.6f,%ld,%.4f,%ld,%.6f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d\n",
                 t, i, fz, enc.count - enc0, z, f[0], f[1], f[2], f[3],
                 adc.ch(0), adc.ch(1), adc.ch(2), adc.ch(3));
        ++i;

        if (feed.ok() && ++bin_n >= udp_div) {
            // P,t,fz,z,enc,f0,f1,f2,f3,fz_min,fz_max,recording,i,gaps,reg_fixes
            char d[256];
            int n = std::snprintf(d, sizeof d, "P,%.4f,%.4f,%.5f,%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%ld,%ld,%ld\n",
                                  t, fz, z, enc.count - enc0, f[0], f[1], f[2], f[3], bin_min, bin_max,
                                  out.ok() ? 1 : 0, i, gaps, reg_fixes);
            if (n > 0) {
                if (dgram.size() + size_t(n) > MAX_DGRAM) dgram_flush();
                dgram.append(d, size_t(n));
                if (++dgram_n >= udp_batch) dgram_flush();
            }
            bin_n = 0; bin_min = 1e9; bin_max = -1e9;
        }

        double now = now_s();
        if (test_corrupt_at >= 0 && t >= test_corrupt_at) {
            test_corrupt_at = -1;
            adc.write_reg(REG_GAIN1, 0x0000);
            std::fprintf(stderr, "\n[%.3f] TEST: GAIN1 forced to 0000 (PGA gain 1)\n", t);
        }
        if (a.reg_check > 0 && now - t_regchk >= a.reg_check) {
            t_regchk = now;
            uint16_t got[3];
            int bad = adc.verify_fix(got);
            if (bad) {
                ++reg_fixes;
                std::fprintf(stderr, "\n[%.3f] ADC registers lost (#%ld): GAIN1 %04x GAIN2 %04x CLOCK %04x, "
                                     "want %04x %04x %04x -- %s\n", t, reg_fixes, got[0], got[1], got[2],
                             adc.want_gain(), adc.want_gain(), adc.want_clock(),
                             (bad & 8) ? "REWRITE FAILED" : "rewritten");
                std::fflush(stderr);
                if (out.ok())
                    emit("# regfix %.6f n %ld gain1 %04x gain2 %04x clock %04x%s\n", t, reg_fixes,
                         got[0], got[1], got[2], (bad & 8) ? " rewrite_failed" : "");
            }
            t_prev = -1;      // the 1-2 samples the read-back cost are not a dropout
        }
        if (a.status > 0 && now - t_status >= a.status) {
            double rate = (i - i_rate) / (now - t_rate);
            i_rate = i; t_rate = now; t_status = now;
            std::fprintf(stderr, "%st=%7.2fs  Fz=%8.3f N  z=%+8.4f m  enc=%7ld  %5.0f sps  gaps %ld  crc %ld %s  %s",
                         a.status_newline ? "" : "\r", t, fz, z, enc.count - enc0, rate, gaps, crc_err,
                         out.ok() ? "REC" : "   ", a.status_newline ? "\n" : "");
            std::fflush(stderr);
        }
    }

    double elapsed = now_s() - t0;
    stop_rec(i, elapsed, gaps, crc_err, rec_min, rec_max);
    dgram_flush();
    feed.close();
    std::fprintf(stderr, "\n%ld samples in %.2f s (%.1f sps, clock expects %ld), %ld gaps, %ld crc errors, Fz %.3f..%.3f N\n",
                 i, elapsed, i / elapsed, long(std::lround(elapsed / period)), gaps, crc_err, fz_min, fz_max);
    return 0;
}
