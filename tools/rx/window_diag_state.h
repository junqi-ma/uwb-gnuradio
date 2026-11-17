// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <vector>
namespace window_diag {
struct Local { int call; long long want, radius, pos; float metric, threshold; };
struct Sfd { int call; long long expected, center, pos; float metric, threshold; };
struct Nsfd { float metric, threshold; long long position; };
inline int timing_call=0, sfd_call=0;
inline double force_cfo=0;
inline bool nominal_period=false;
inline std::vector<Local> locals;
inline std::vector<Sfd> sfds;
inline std::vector<Nsfd> nsfds;
inline void local(long long w,long long r,float m,long long p,float t) {
    locals.push_back({timing_call,w,r,p,m,t});
}
inline void sfd(long long e,long long c,float m,long long p,float t) {
    sfds.push_back({sfd_call,e,c,p,m,t});
}
}
