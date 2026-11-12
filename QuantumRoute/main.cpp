// Quantum Route Lab - Dynamic Multi-Criteria Pathfinding Simulator (C++20)
// Build: g++ -std=c++20 -O2 main.cpp -o quantum_route
// Run:   ./quantum_route [seed]      (type "help" inside)
// Grid is 35x21, cell index = y*35 + x.  ANSI terminal recommended ("color off" disables).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Ansi {
inline bool on = true;
inline std::string fg(int c) { return on ? "\033[38;5;" + std::to_string(c) + "m" : ""; }
inline std::string bg(int c) { return on ? "\033[48;5;" + std::to_string(c) + "m" : ""; }
inline std::string bold() { return on ? "\033[1m" : ""; }
inline std::string rst() { return on ? "\033[0m" : ""; }
inline std::string eol() { return on ? "\033[K" : ""; }
inline std::string up(int n) { return on ? "\033[" + std::to_string(n) + "A\r" : ""; }
}  // namespace Ansi

constexpr int C = 35, R = 21, N = C * R;
constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double SQ2 = 1.41421356237309505;
constexpr double PI = 3.14159265358979323846;

// ---------------------------------------------------------------- DSU
class DSU {
    std::vector<int> p_, r_;
    int comps_ = 0;
public:
    void reset(int n) { p_.resize(n); std::iota(p_.begin(), p_.end(), 0); r_.assign(n, 0); comps_ = n; }
    int find(int x) {
        int r = x;
        while (p_[r] != r) r = p_[r];
        while (p_[x] != r) { int n = p_[x]; p_[x] = r; x = n; }  // path compression
        return r;
    }
    void unite(int a, int b) {
        a = find(a); b = find(b);
        if (a == b) return;
        if (r_[a] < r_[b]) std::swap(a, b);                      // union by rank
        p_[b] = a;
        if (r_[a] == r_[b]) ++r_[a];
        --comps_;
    }
    bool connected(int a, int b) { return find(a) == find(b); }
    int components() const { return comps_; }
};

// ---------------------------------------------------------------- Segment tree (traffic factors)
class SegTree {
    int n_ = 0;
    std::vector<double> s_, m_;
    void pull(int i) { s_[i] = s_[2 * i] + s_[2 * i + 1]; m_[i] = std::max(m_[2 * i], m_[2 * i + 1]); }
public:
    void init(int n, double v) {
        n_ = n; s_.assign(2 * n, 0.0); m_.assign(2 * n, -INF);
        for (int i = 0; i < n; ++i) s_[n + i] = m_[n + i] = v;
        for (int i = n - 1; i > 0; --i) pull(i);
    }
    void set(int p, double v) { int i = p + n_; s_[i] = m_[i] = v; for (i >>= 1; i > 0; i >>= 1) pull(i); }  // O(log K)
    double get(int p) const { return s_[p + n_]; }
    double sum(int l, int r) const {  // [l, r)
        double a = 0;
        for (l += n_, r += n_; l < r; l >>= 1, r >>= 1) { if (l & 1) a += s_[l++]; if (r & 1) a += s_[--r]; }
        return a;
    }
    double max(int l, int r) const {
        double a = -INF;
        for (l += n_, r += n_; l < r; l >>= 1, r >>= 1) { if (l & 1) a = std::max(a, m_[l++]); if (r & 1) a = std::max(a, m_[--r]); }
        return a;
    }
};

// ---------------------------------------------------------------- World (dynamic graph)
struct Weights { double wd = 1, wt = 1, wr = 1; };
struct Totals { double d = 0, t = 0, r = 0, c = 0; };

struct World {
    std::vector<char> wall = std::vector<char>(N, 0);
    std::vector<float> risk = std::vector<float>(N, 0.f);
    std::unordered_set<long long> cutEdges;  // blocked edges (u-v)
    SegTree traffic;
    DSU dsu;
    bool dir8 = false;

    World() { traffic.init(N, 1.0); }
    static long long ekey(int a, int b) { if (a > b) std::swap(a, b); return static_cast<long long>(a) * N + b; }
    bool edgeCut(int a, int b) const { return cutEdges.count(ekey(a, b)) > 0; }

    template <class F>
    void eachNb(int i, F fn) const {
        static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        int x = i % C, y = i / C, cnt = dir8 ? 8 : 4;
        for (int k = 0; k < cnt; ++k) {
            int nx = x + DX[k], ny = y + DY[k];
            if (nx < 0 || ny < 0 || nx >= C || ny >= R) continue;
            int j = ny * C + nx;
            if (wall[j] || edgeCut(i, j)) continue;
            if (DX[k] && DY[k] && (wall[y * C + nx] || wall[ny * C + x])) continue;  // no corner cutting
            fn(j, (DX[k] && DY[k]) ? SQ2 : 1.0);
        }
    }
    void rebuildDSU() {
        dsu.reset(N);
        for (int i = 0; i < N; ++i) if (!wall[i]) eachNb(i, [&](int j, double) { dsu.unite(i, j); });
    }
    // cost of stepping INTO cell j:  wd*dist + wt*(dist/speed) + wr*risk,  speed = 1/(traffic*noise)
    double stepCost(int j, double d, const Weights& w, const std::vector<double>* nz) const {
        double f = traffic.get(j) * (nz ? (*nz)[j] : 1.0);
        return w.wd * d + w.wt * d * f + w.wr * risk[j];
    }
    Totals stats(const std::vector<int>& p, const Weights& w, const std::vector<double>* nz = nullptr) const {
        Totals t;
        for (size_t i = 1; i < p.size(); ++i) {
            int a = p[i - 1], b = p[i];
            double d = (a % C != b % C && a / C != b / C) ? SQ2 : 1.0;
            double f = traffic.get(b) * (nz ? (*nz)[b] : 1.0);
            t.d += d; t.t += d * f; t.r += risk[b]; t.c += stepCost(b, d, w, nz);
        }
        return t;
    }
    double unitCost(const Weights& w, const std::vector<double>* nz) const {
        double m = INF;
        for (int i = 0; i < N; ++i) if (!wall[i]) {
            double v = w.wd + w.wt * traffic.get(i) * (nz ? (*nz)[i] : 1.0) + w.wr * risk[i] / SQ2;
            m = std::min(m, v);
        }
        return std::isfinite(m) ? m : 0.0;
    }
};

// ---------------------------------------------------------------- Search (Dijkstra / A*)
struct Result {
    bool found = false;
    std::vector<int> path;
    size_t popped = 0, expanded = 0;
    double micros = 0;
    Totals tot;
    std::vector<int> ex, ds;  // expansion index, discovery index (for animation)
};

Result search(const World& g, int s, int t, const Weights& w, bool useH, bool euclid,
              const std::vector<double>* nz) {
    Result res;
    std::vector<double> gs(N, INF);
    std::vector<int> par(N, -1);
    std::vector<char> closed(N, 0);
    res.ex.assign(N, -1); res.ds.assign(N, -1);
    double u = useH ? g.unitCost(w, nz) : 0.0;
    int tx = t % C, ty = t / C;
    auto h = [&](int v) -> double {
        if (!useH) return 0.0;
        double dx = std::abs(v % C - tx), dy = std::abs(v / C - ty);
        double d = euclid ? std::hypot(dx, dy) : (g.dir8 ? std::max(dx, dy) : dx + dy);
        return d * u;  // admissible: u is the cheapest possible cost per unit distance
    };
    using QI = std::pair<double, int>;
    std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
    gs[s] = 0; res.ds[s] = 0; pq.push({h(s), s});
    while (!pq.empty()) {
        int v = pq.top().second; pq.pop();
        ++res.popped;
        if (closed[v]) continue;
        closed[v] = 1; res.ex[v] = static_cast<int>(res.expanded++);
        if (v == t) { res.found = true; break; }
        g.eachNb(v, [&](int j, double d) {
            if (closed[j]) return;
            double ng = gs[v] + g.stepCost(j, d, w, nz);
            if (ng < gs[j]) {
                gs[j] = ng; par[j] = v;
                if (res.ds[j] < 0) res.ds[j] = static_cast<int>(res.expanded);
                pq.push({ng + h(j), j});
            }
        });
    }
    if (res.found) {
        for (int v = t; v != -1; v = par[v]) res.path.push_back(v);
        std::reverse(res.path.begin(), res.path.end());
        res.tot = g.stats(res.path, w);
    }
    return res;
}

// ---------------------------------------------------------------- Quantum-inspired selection
struct Cand { std::vector<int> path; double det = 0, mean = 0, sd = 0, score = 0, amp = 0, prob = 0; };
struct QReport { bool ok = false; std::vector<Cand> c; int chosen = -1, iters = 0; size_t popped = 0, expanded = 0; double micros = 0; Totals tot; };

QReport quantumSelect(const World& g, int s, int t, const Weights& w, bool euclid, double sigma, std::mt19937& rng) {
    QReport q;
    auto t0 = std::chrono::steady_clock::now();
    std::normal_distribution<double> nd(0, 1);
    auto noise = [&](double sg) { std::vector<double> v(N); for (auto& x : v) x = std::exp(sg * nd(rng)); return v; };
    const int NC = 12, NS = 48;
    for (int k = 0; k < NC; ++k) {                        // 1. superposition of candidate trajectories
        std::vector<double> nz;
        if (k) nz = noise(sigma * 1.8);
        Result r = search(g, s, t, w, true, euclid, k ? &nz : nullptr);
        q.popped += r.popped; q.expanded += r.expanded;
        if (!r.found) { if (!k) return q; continue; }
        bool dup = false;
        for (auto& c : q.c) if (c.path == r.path) { dup = true; break; }
        if (!dup) { Cand c; c.path = r.path; c.det = r.tot.c; q.c.push_back(std::move(c)); }
    }
    int M = static_cast<int>(q.c.size());
    std::vector<double> sum(M, 0), sq(M, 0);
    for (int sc = 0; sc < NS; ++sc) {                     // 2. Monte-Carlo amplitude evaluation
        auto nz = noise(sigma);
        for (int i = 0; i < M; ++i) { double x = g.stats(q.c[i].path, w, &nz).c; sum[i] += x; sq[i] += x * x; }
    }
    int best = 0;
    for (int i = 0; i < M; ++i) {
        q.c[i].mean = sum[i] / NS;
        q.c[i].sd = std::sqrt(std::max(0.0, sq[i] / NS - q.c[i].mean * q.c[i].mean));
        q.c[i].score = q.c[i].mean + 0.5 * q.c[i].sd;
        if (q.c[i].score < q.c[best].score) best = i;
    }
    std::vector<double> amp(M, 1.0 / std::sqrt(static_cast<double>(M)));
    q.iters = M > 1 ? std::max(1, static_cast<int>(std::lround(PI / 4 * std::sqrt(static_cast<double>(M))))) : 0;
    for (int k = 0; k < q.iters; ++k) {                   // 3. Grover-style oracle + diffusion
        amp[best] = -amp[best];
        double m = std::accumulate(amp.begin(), amp.end(), 0.0) / M;
        for (auto& a : amp) a = 2 * m - a;
    }
    double nrm = 0; for (double a : amp) nrm += a * a;
    int ch = 0;
    for (int i = 0; i < M; ++i) {
        q.c[i].amp = amp[i]; q.c[i].prob = amp[i] * amp[i] / nrm;
        if (q.c[i].prob > q.c[ch].prob + 1e-12) ch = i;   // 4. collapse
    }
    q.chosen = ch; q.ok = true; q.tot = g.stats(q.c[ch].path, w);
    q.micros = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    return q;
}

// ---------------------------------------------------------------- Simulator
struct Overlay {
    std::vector<char> expl, front, cand, path;
    Overlay() : expl(N, 0), front(N, 0), cand(N, 0), path(N, 0) {}
};

class Sim {
    World g;
    Weights w;
    std::mt19937 rng;
    int start = 1 * C + 1, goal = (R - 2) * C + (C - 2), agent = start;
    std::vector<int> plan;
    int ver = 0, planVer = -1;
    bool euclid = false, anim = true, useQ = true;
    int frameMs = 25;
    double sigma = 0.3, travelled = 0;

    static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
    bool valid(int v) const { return v >= 0 && v < N; }
    int rnd(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
    double u01() { return std::uniform_real_distribution<double>(0, 1)(rng); }
    void touch() { ++ver; }
    void blob(int cx, int cy, int r, const std::function<void(int, double)>& fn) {
        for (int y = 0; y < R; ++y) for (int x = 0; x < C; ++x) {
            double d = std::hypot(x - cx, y - cy);
            if (d <= r) fn(y * C + x, 1.0 - d / r);
        }
    }

    // ---- presets
    void generate(const std::string& kind) {
        g = World(); plan.clear(); planVer = -1; travelled = 0;
        start = 1 * C + 1; goal = (R - 2) * C + (C - 2); agent = start;
        auto tr = [&](int i, double t) { if (!g.wall[i]) g.traffic.set(i, 1 + t); };
        auto rk = [&](int i, double v) { if (!g.wall[i]) g.risk[i] = static_cast<float>(std::max<double>(g.risk[i], std::round(v))); };
        if (kind == "city") {
            for (int y = 0; y < R; ++y) for (int x = 0; x < C; ++x) if (x % 6 && y % 5 && u01() < 0.93) g.wall[y * C + x] = 1;
            for (int k = 0; k < 6; ++k) blob(rnd(2, C - 3), rnd(2, R - 3), rnd(2, 4), [&](int i, double t) { tr(i, t * 5); });
            for (int k = 0; k < 4; ++k) blob(rnd(2, C - 3), rnd(2, R - 3), rnd(2, 3), [&](int i, double t) { rk(i, 2 + t * 7); });
        } else if (kind == "maze") {
            std::fill(g.wall.begin(), g.wall.end(), 1);
            std::vector<int> st{1 * C + 1}; g.wall[st[0]] = 0;
            while (!st.empty()) {
                int v = st.back(), x = v % C, y = v / C;
                std::vector<std::pair<int, int>> o;
                for (auto [a, b] : std::vector<std::pair<int, int>>{{2, 0}, {-2, 0}, {0, 2}, {0, -2}})
                    if (x + a > 0 && y + b > 0 && x + a < C - 1 && y + b < R - 1 && g.wall[(y + b) * C + x + a]) o.push_back({a, b});
                if (o.empty()) { st.pop_back(); continue; }
                auto [a, b] = o[rnd(0, static_cast<int>(o.size()) - 1)];
                g.wall[(y + b / 2) * C + x + a / 2] = 0; g.wall[(y + b) * C + x + a] = 0;
                st.push_back((y + b) * C + x + a);
            }
            for (int k = 0; k < 40; ++k) g.wall[rnd(1, R - 2) * C + rnd(1, C - 2)] = 0;
            for (int i = 0; i < N; ++i) if (!g.wall[i] && u01() < 0.08) g.risk[i] = static_cast<float>(rnd(2, 6));
        } else if (kind == "field") {
            for (int k = 0; k < 9; ++k) blob(rnd(3, C - 4), rnd(3, R - 4), rnd(1, 3), [&](int i, double) { g.wall[i] = 1; });
            for (int k = 0; k < 5; ++k) blob(rnd(3, C - 4), rnd(3, R - 4), rnd(3, 5), [&](int i, double t) { rk(i, 1 + t * 8); });
            for (int k = 0; k < 4; ++k) blob(rnd(3, C - 4), rnd(3, R - 4), rnd(3, 5), [&](int i, double t) { tr(i, t * 4); });
        } else {  // disaster
            for (int k = 0; k < 22; ++k) blob(rnd(1, C - 2), rnd(1, R - 2), rnd(1, 2), [&](int i, double) { g.wall[i] = 1; });
            for (int k = 0; k < 9; ++k) blob(rnd(2, C - 3), rnd(2, R - 3), rnd(3, 6), [&](int i, double t) { rk(i, 2 + t * 8); });
            for (int k = 0; k < 7; ++k) blob(rnd(2, C - 3), rnd(2, R - 3), rnd(2, 5), [&](int i, double t) { tr(i, t * 6); });
        }
        g.wall[start] = g.wall[goal] = 0;
        g.rebuildDSU();
        for (int guard = 0; !g.dsu.connected(start, goal) && guard < 2000; ++guard) { g.wall[rnd(0, N - 1)] = 0; g.rebuildDSU(); }
        touch();
    }

    // ---- rendering
    std::string cell(int i, const Overlay& ov) const {
        using namespace Ansi;
        double f = g.traffic.get(i);
        bool plainMode = !on;
        if (i == agent) return plainMode ? "@ " : bg(226) + fg(16) + bold() + "@ " + rst();
        if (i == goal) return plainMode ? "G " : bg(46) + fg(16) + bold() + "G " + rst();
        if (i == start) return plainMode ? "S " : bg(255) + fg(16) + "S " + rst();
        if (g.wall[i]) return plainMode ? "##" : bg(52) + fg(196) + bold() + "XX" + rst();
        if (ov.path[i]) return plainMode ? "**" : bg(34) + fg(226) + bold() + "<>" + rst();
        if (ov.cand[i]) return plainMode ? "??" : bg(93) + fg(225) + "::" + rst();
        std::string ch = g.risk[i] > 0 ? "!" : (f > 1.05 ? "~" : " ");
        if (g.risk[i] > 0 && f > 1.05) ch = "!";
        ch += " ";
        if (plainMode) return ov.expl[i] ? ". " : (ov.front[i] ? "o " : ch);
        int base = 235;
        if (f > 1.05) base = f > 4 ? 202 : (f > 2.5 ? 166 : 130);
        if (ov.expl[i]) return bg(30) + fg(51) + bold() + ch + rst();
        if (ov.front[i]) return bg(45) + fg(16) + ch + rst();
        return bg(base) + fg(g.risk[i] > 0 ? 205 : 255) + (g.risk[i] > 0 ? bold() : "") + ch + rst();
    }
    void draw(const Overlay& ov, const std::string& status, bool overwrite) const {
        std::ostringstream o;
        if (overwrite) o << Ansi::up(R + 2);
        o << Ansi::bold() << status << Ansi::rst() << Ansi::eol() << "\n    ";
        for (int x = 0; x < C; ++x) o << (x % 5 == 0 ? std::to_string(x % 100) + std::string(x % 5 == 0 && x >= 10 ? 0 : 1, ' ') : std::string("  ")).substr(0, 2);
        o << Ansi::eol() << "\n";
        for (int y = 0; y < R; ++y) {
            o << std::setw(3) << y * C << " ";
            for (int x = 0; x < C; ++x) o << cell(y * C + x, ov);
            o << Ansi::eol() << "\n";
        }
        std::cout << o.str() << std::flush;
    }
    void mark(Overlay& ov, const std::vector<int>& p, bool cand) const { for (int v : p) (cand ? ov.cand : ov.path)[v] = 1; }
    Overlay base() const { Overlay ov; if (!plan.empty()) mark(ov, plan, false); return ov; }
    void show(const std::string& title = "") const {
        std::ostringstream st;
        st << "agent=" << agent << " goal=" << goal << " w(d,t,r)=" << w.wd << "," << w.wt << "," << w.wr
           << " " << (g.dir8 ? "8-dir" : "4-dir") << " " << (euclid ? "Euclid" : "Manhattan");
        if (!title.empty()) st << " | " << title;
        draw(base(), st.str(), false);
    }
    void legend() const {
        using namespace Ansi;
        std::cout << "  " << bg(30) << fg(51) << " explored " << rst() << " " << bg(45) << fg(16) << " frontier " << rst() << " "
                  << bg(93) << fg(225) << " quantum cand. " << rst() << " " << bg(34) << fg(226) << " route " << rst() << " "
                  << bg(52) << fg(196) << " blocked " << rst() << " " << bg(166) << " traffic " << rst() << " " << fg(205) << "! risk" << rst()
                  << "\n  index = row*" << C << " + col (row labels on the left show the first index of each row)\n";
    }

    void animateSearch(const Result& r, const std::string& title) const {
        Overlay ov;
        int total = static_cast<int>(r.expanded), frames = std::max(1, std::min(total, 40)), batch = (total + frames - 1) / frames;
        auto fill = [&](int k) {
            for (int v = 0; v < N; ++v) {
                ov.expl[v] = (r.ex[v] >= 0 && r.ex[v] < k);
                ov.front[v] = (!ov.expl[v] && r.ds[v] >= 0 && r.ds[v] <= k);
            }
        };
        bool drew = false;
        if (anim && Ansi::on)
            for (int k = batch; k < total; k += batch) { fill(k); draw(ov, title + " expanded " + std::to_string(k) + "/" + std::to_string(total), drew); drew = true; sleepMs(frameMs); }
        fill(total); mark(ov, r.path, false);
        draw(ov, title + " expanded " + std::to_string(total) + "/" + std::to_string(total) + " -> route found", drew);
    }
    void animateQuantum(const QReport& q) const {
        Overlay ov; bool drew = false;
        for (size_t i = 0; i < q.c.size(); ++i) {
            mark(ov, q.c[i].path, true);
            if (anim && Ansi::on) { draw(ov, "Quantum superposition: candidate " + std::to_string(i + 1) + "/" + std::to_string(q.c.size()), drew); drew = true; sleepMs(frameMs * 4); }
        }
        mark(ov, q.c[q.chosen].path, false);
        draw(ov, "Wavefunction collapsed -> candidate #" + std::to_string(q.chosen), drew);
    }
    void printQuantum(const QReport& q) const {
        std::cout << "\nQuantum register: " << q.c.size() << " superposed paths, sigma=" << sigma << ", Grover iterations=" << q.iters << "\n";
        std::cout << std::left << std::setw(4) << " #" << std::setw(7) << "Hops" << std::setw(11) << "DetCost" << std::setw(11) << "E[cost]"
                  << std::setw(9) << "StdDev" << std::setw(10) << "Score" << std::setw(10) << "Amp" << "Prob\n";
        for (size_t i = 0; i < q.c.size(); ++i) {
            const Cand& c = q.c[i];
            std::ostringstream o;
            o << " " << std::left << std::setw(3) << i << std::setw(7) << c.path.size() - 1 << std::fixed << std::setprecision(2)
              << std::setw(11) << c.det << std::setw(11) << c.mean << std::setw(9) << c.sd << std::setw(10) << c.score
              << std::setprecision(3) << std::setw(10) << c.amp << std::setprecision(1) << c.prob * 100 << "%";
            if (static_cast<int>(i) == q.chosen) o << "  <== collapsed";
            if (static_cast<int>(i) == q.chosen) std::cout << Ansi::fg(165) << Ansi::bold() << o.str() << Ansi::rst() << "\n";
            else std::cout << o.str() << "\n";
        }
    }
    struct Row { std::string name; double us; size_t popped, expanded; Totals t; };
    void table(const std::vector<Row>& rows) const {
        auto sep = [] { std::cout << "+" << std::string(18, '-') << "+" << std::string(11, '-') << "+" << std::string(8, '-') << "+" << std::string(10, '-') << "+" << std::string(11, '-') << "+" << std::string(9, '-') << "+" << std::string(9, '-') << "+" << std::string(8, '-') << "+\n"; };
        sep();
        std::cout << "| " << std::left << std::setw(16) << "Algorithm" << " | " << std::setw(9) << "Time(us)" << " | " << std::setw(6) << "Popped" << " | " << std::setw(8) << "Expanded"
                  << " | " << std::setw(9) << "Cost" << " | " << std::setw(7) << "Dist" << " | " << std::setw(7) << "Time" << " | " << std::setw(6) << "Risk" << " |\n";
        sep();
        for (auto& r : rows)
            std::cout << "| " << std::left << std::setw(16) << r.name << " | " << std::right << std::fixed << std::setprecision(1) << std::setw(9) << r.us << " | " << std::setw(6) << r.popped
                      << " | " << std::setw(8) << r.expanded << " | " << std::setprecision(2) << std::setw(9) << r.t.c << " | " << std::setw(7) << r.t.d << " | " << std::setw(7) << r.t.t
                      << " | " << std::setw(6) << r.t.r << " |\n";
        sep();
        std::cout << std::left;
    }
    Result profile(bool useH, int reps = 25) const {
        Result r;
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < reps; ++i) r = search(g, agent, goal, w, useH, euclid, nullptr);
        r.micros = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / reps;
        return r;
    }
    bool replan() {
        g.rebuildDSU();
        if (!g.dsu.connected(agent, goal)) { plan.clear(); return false; }
        Result r = search(g, agent, goal, w, true, euclid, nullptr);
        if (!r.found) { plan.clear(); return false; }
        plan = r.path; planVer = ver; return true;
    }

public:
    explicit Sim(unsigned seed) : rng(seed) { generate("city"); }

    void help() const {
        std::cout << "Commands:\n"
                  << "  run                        DSU check, Dijkstra vs A* vs Quantum collapse + stats table\n"
                  << "  step                       move agent one node along the optimal route\n"
                  << "  live [n]                   n ticks of fluctuating traffic: re-plan + move each tick (default 15)\n"
                  << "  block <u> <v>              cut edge between adjacent cells u,v (DSU rebuilt)   | unblock <u> <v>\n"
                  << "  wall <idx> | unwall <idx>  block / free a whole cell\n"
                  << "  traffic <u> <v> <factor>   congestion on edge u-v via segment tree (0.1..100)\n"
                  << "  traffic <idx> <factor>     congestion on one cell\n"
                  << "  risk <idx> <value>         hazard level of a cell (0..20)\n"
                  << "  weights <w1> <w2> <w3>     w_dist, w_time, w_risk\n"
                  << "  query <l> <r>              segment-tree range query on cell indices [l,r]\n"
                  << "  preset city|maze|field|disaster\n"
                  << "  src <idx> | dst <idx>      move start / destination\n"
                  << "  dir8 on|off | heur manh|euclid | quantum on|off | sigma <0..1>\n"
                  << "  anim on|off|<ms> | color on|off | show | reset | help | quit\n";
    }
    void cmdShow() const { show(); legend(); }
    void cmdRun() {
        if (agent == goal) { std::cout << "Agent is already at the goal.\n"; return; }
        g.rebuildDSU();
        bool conn = g.dsu.connected(agent, goal);
        std::cout << "[DSU] components=" << g.dsu.components() << " connected(" << agent << "," << goal << ")=" << (conn ? "true" : "false") << "\n";
        if (!conn) { std::cout << Ansi::fg(196) << "No route exists - search skipped." << Ansi::rst() << "\n"; plan.clear(); show("NO ROUTE"); return; }
        Result D = profile(false), A = profile(true);
        animateSearch(D, "Dijkstra"); std::cout << "\n";
        animateSearch(A, "A*      "); std::cout << "\n";
        std::vector<Row> rows{{"Dijkstra", D.micros, D.popped, D.expanded, D.tot}, {euclid ? "A* (Euclid)" : "A* (Manhattan)", A.micros, A.popped, A.expanded, A.tot}};
        QReport q;
        if (useQ) { q = quantumSelect(g, agent, goal, w, euclid, sigma, rng); if (q.ok) rows.push_back({"Quantum collapse", q.micros, q.popped, q.expanded, q.tot}); }
        table(rows);
        std::cout << "A* vs Dijkstra cost match: " << (std::abs(A.tot.c - D.tot.c) < 1e-6 ? "yes" : "NO") << " | A* expanded " << std::fixed << std::setprecision(1)
                  << 100.0 * (1.0 - static_cast<double>(A.expanded) / static_cast<double>(D.expanded)) << "% fewer nodes\n";
        if (q.ok) {
            printQuantum(q); std::cout << "\n"; animateQuantum(q);
            plan = q.c[q.chosen].path;
        } else { plan = A.path; show("A* route"); }
        planVer = ver;
        legend();
    }
    void cmdStep(bool quiet = false) {
        if (agent == goal) { std::cout << "Already delivered.\n"; return; }
        if (plan.size() < 2 || planVer != ver || plan[0] != agent) {
            if (!replan()) { std::cout << Ansi::fg(196) << "Goal unreachable from " << agent << " (DSU). Agent holds." << Ansi::rst() << "\n"; return; }
            std::cout << "[replan] graph changed -> new A* route, cost " << std::fixed << std::setprecision(2) << g.stats(plan, w).c << "\n";
        }
        int nxt = plan[1];
        travelled += g.stats({agent, nxt}, w).c;
        std::cout << "Agent " << agent << " -> " << nxt << "\n";
        agent = nxt; plan.erase(plan.begin());
        if (agent == goal) { std::cout << Ansi::fg(46) << Ansi::bold() << "DELIVERED!" << Ansi::rst() << " travelled composite cost = " << std::fixed << std::setprecision(2) << travelled << "\n"; plan.clear(); }
        if (!quiet) show();
    }
    void cmdLive(int n) {
        if (plan.empty() && !replan()) { std::cout << "No route.\n"; return; }
        for (int k = 0; k < n && agent != goal; ++k) {
            for (int j = 0; j < 8; ++j) {
                int i = rnd(0, N - 1);
                if (g.wall[i] || i == agent) continue;
                g.traffic.set(i, std::clamp(g.traffic.get(i) + (u01() < 0.5 ? -1.2 : 1.6), 1.0, 8.0));
            }
            touch();
            if (!replan()) { std::cout << "Route lost.\n"; return; }
            int nxt = plan[1]; travelled += g.stats({agent, nxt}, w).c; agent = nxt; plan.erase(plan.begin()); planVer = ver;
            if (agent == goal) plan.clear();
            std::ostringstream st; st << "LIVE tick " << k + 1 << "/" << n << " agent=" << agent;
            draw(base(), st.str(), k > 0);
            sleepMs(std::max(frameMs * 4, 80));
        }
        if (agent == goal) std::cout << "DELIVERED under live traffic. Composite cost " << std::fixed << std::setprecision(2) << travelled << "\n";
    }
    void cmdBlock(int u, int v, bool b) {
        if (!valid(u) || !valid(v)) { std::cout << "Index out of range 0.." << N - 1 << "\n"; return; }
        int dx = std::abs(u % C - v % C), dy = std::abs(u / C - v / C);
        if (std::max(dx, dy) != 1 || (!g.dir8 && dx + dy != 1)) { std::cout << "Cells are not adjacent.\n"; return; }
        if (b) g.cutEdges.insert(World::ekey(u, v)); else g.cutEdges.erase(World::ekey(u, v));
        g.rebuildDSU(); touch();
        std::cout << "Edge " << u << "-" << v << (b ? " BLOCKED" : " RESTORED") << ". DSU components=" << g.dsu.components() << " connected(" << agent << "," << goal << ")=" << (g.dsu.connected(agent, goal) ? "true" : "false") << "\n";
        show();
    }
    void cmdWall(int i, bool b) {
        if (!valid(i) || i == agent || i == goal) { std::cout << "Invalid cell.\n"; return; }
        g.wall[i] = b; g.rebuildDSU(); touch();
        std::cout << "Cell " << i << (b ? " blocked" : " freed") << ". DSU components=" << g.dsu.components() << "\n"; show();
    }
    void cmdTraffic(const std::vector<double>& a) {
        auto setf = [&](int i, double f) { if (!valid(i)) return false; g.traffic.set(i, f); return true; };
        if (a.size() == 3) {
            int u = static_cast<int>(a[0]), v = static_cast<int>(a[1]); double f = a[2];
            if (!valid(u) || !valid(v) || f < 0.1 || f > 100) { std::cout << "Bad arguments (factor 0.1..100).\n"; return; }
            if (std::max(std::abs(u % C - v % C), std::abs(u / C - v / C)) != 1) { std::cout << "Cells are not adjacent.\n"; return; }
            setf(u, f); setf(v, f);
            std::cout << "Edge " << u << "-" << v << " traffic factor -> " << f << " (segment-tree point updates, O(log " << N << "))\n";
        } else if (a.size() == 2) {
            int i = static_cast<int>(a[0]); double f = a[1];
            if (!valid(i) || f < 0.1 || f > 100) { std::cout << "Bad arguments.\n"; return; }
            setf(i, f); std::cout << "Cell " << i << " traffic factor -> " << f << "\n";
        } else { std::cout << "Usage: traffic <u> <v> <factor> | traffic <idx> <factor>\n"; return; }
        touch();
        std::cout << std::fixed << std::setprecision(2) << "Network delay index (segment-tree sum - K): " << g.traffic.sum(0, N) - N << "\n";
        show();
    }
    void cmdRisk(int i, double v) { if (!valid(i) || v < 0 || v > 20) { std::cout << "Bad arguments.\n"; return; } g.risk[i] = static_cast<float>(v); touch(); show(); }
    void cmdWeights(double a, double b, double c) {
        if (a < 0 || b < 0 || c < 0 || a + b + c <= 0) { std::cout << "Weights must be >= 0 and not all zero.\n"; return; }
        w = {a, b, c}; touch(); std::cout << "Priorities: w_dist=" << a << " w_time=" << b << " w_risk=" << c << " (use 'run' to re-plan)\n";
    }
    void cmdQuery(int l, int r) {
        if (l < 0 || r >= N || l > r) { std::cout << "Need 0 <= l <= r < " << N << "\n"; return; }
        double s = g.traffic.sum(l, r + 1); int len = r - l + 1;
        std::cout << std::fixed << std::setprecision(3) << "Cells [" << l << "," << r << "]: total delay=" << s - len << " avg factor=" << s / len << " max factor=" << g.traffic.max(l, r + 1) << "\n";
    }
    void cmdPreset(const std::string& k) {
        if (k != "city" && k != "maze" && k != "field" && k != "disaster") { std::cout << "Use: preset city|maze|field|disaster\n"; return; }
        generate(k); std::cout << "Generated '" << k << "'. DSU connected: " << (g.dsu.connected(agent, goal) ? "yes" : "no") << "\n"; show(); legend();
    }
    void cmdSrc(int v) { if (!valid(v) || v == goal || g.wall[v]) { std::cout << "Invalid.\n"; return; } start = agent = v; plan.clear(); travelled = 0; touch(); show(); }
    void cmdDst(int v) { if (!valid(v) || v == agent || g.wall[v]) { std::cout << "Invalid.\n"; return; } goal = v; plan.clear(); touch(); show(); }
    void cmdReset() { agent = start; plan.clear(); travelled = 0; touch(); show("agent reset"); }

    void loop() {
        std::cout << Ansi::fg(51) << Ansi::bold() << "=== QUANTUM ROUTE LAB - dynamic multi-criteria pathfinding ===" << Ansi::rst() << "\nType 'help' for commands.\n";
        cmdShow();
        std::string line;
        while (true) {
            std::cout << Ansi::fg(51) << "quantum-route> " << Ansi::rst() << std::flush;
            if (!std::getline(std::cin, line)) break;
            std::istringstream is(line);
            std::string cmd;
            if (!(is >> cmd)) continue;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            std::vector<std::string> a; std::string t;
            while (is >> t) a.push_back(t);
            auto num = [&](size_t i, double& out) { if (i >= a.size()) return false; try { size_t p; out = std::stod(a[i], &p); return p == a[i].size(); } catch (...) { return false; } };
            double x, y, z;
            if (cmd == "quit" || cmd == "exit" || cmd == "q") break;
            else if (cmd == "help" || cmd == "?") help();
            else if (cmd == "show") cmdShow();
            else if (cmd == "run") cmdRun();
            else if (cmd == "step") cmdStep();
            else if (cmd == "reset") cmdReset();
            else if (cmd == "live") { double n = 15; num(0, n); cmdLive(static_cast<int>(std::clamp(n, 1.0, 500.0))); }
            else if ((cmd == "block" || cmd == "unblock") && num(0, x) && num(1, y)) cmdBlock(static_cast<int>(x), static_cast<int>(y), cmd == "block");
            else if ((cmd == "wall" || cmd == "unwall") && num(0, x)) cmdWall(static_cast<int>(x), cmd == "wall");
            else if (cmd == "traffic") { std::vector<double> v; double d; for (size_t i = 0; num(i, d); ++i) v.push_back(d); cmdTraffic(v); }
            else if (cmd == "risk" && num(0, x) && num(1, y)) cmdRisk(static_cast<int>(x), y);
            else if (cmd == "weights" && num(0, x) && num(1, y) && num(2, z)) cmdWeights(x, y, z);
            else if (cmd == "query" && num(0, x) && num(1, y)) cmdQuery(static_cast<int>(x), static_cast<int>(y));
            else if (cmd == "preset" && !a.empty()) cmdPreset(a[0]);
            else if (cmd == "src" && num(0, x)) cmdSrc(static_cast<int>(x));
            else if (cmd == "dst" && num(0, x)) cmdDst(static_cast<int>(x));
            else if (cmd == "dir8" && !a.empty()) { g.dir8 = (a[0] == "on"); g.rebuildDSU(); touch(); std::cout << (g.dir8 ? "8" : "4") << "-direction movement.\n"; }
            else if (cmd == "heur" && !a.empty()) { euclid = (a[0][0] == 'e'); touch(); std::cout << "Heuristic: " << (euclid ? "Euclidean" : "Manhattan") << "\n"; }
            else if (cmd == "quantum" && !a.empty()) { useQ = (a[0] == "on"); std::cout << "Quantum filter " << (useQ ? "on" : "off") << "\n"; }
            else if (cmd == "sigma" && num(0, x) && x >= 0 && x <= 1) { sigma = x; std::cout << "sigma = " << sigma << "\n"; }
            else if (cmd == "anim" && !a.empty()) { if (a[0] == "on") anim = true; else if (a[0] == "off") anim = false; else if (num(0, x)) frameMs = static_cast<int>(std::clamp(x, 0.0, 2000.0)); std::cout << "anim " << (anim ? "on" : "off") << ", " << frameMs << " ms\n"; }
            else if (cmd == "color" && !a.empty()) { Ansi::on = (a[0] == "on"); std::cout << "color " << (Ansi::on ? "on" : "off") << "\n"; }
            else std::cout << "Unknown or malformed command '" << cmd << "'. Type 'help'.\n";
        }
        std::cout << "Goodbye.\n";
    }
};

int main(int argc, char** argv) {
    unsigned seed = 2024;
    if (argc > 1) { try { seed = static_cast<unsigned>(std::stoul(argv[1])); } catch (...) {} }
    Sim sim(seed);
    sim.loop();
    return 0;
}
