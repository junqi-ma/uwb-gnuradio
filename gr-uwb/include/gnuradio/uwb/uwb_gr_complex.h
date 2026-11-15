/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * gr_complex alias without <gnuradio/gr_complex.h>.
 *
 * GNU Radio defines `typedef std::complex<float> gr_complex;` in the global
 * namespace.  Redefining a typedef-name to the same type is legal C++, so
 * including this header together with the real <gnuradio/gr_complex.h> is
 * harmless; native (GNU-Radio-free) code uses this header instead.
 */

#pragma once

#include <complex>

typedef std::complex<float> gr_complex;
