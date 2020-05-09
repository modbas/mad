/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * carlocate LTTNG configuration
  *
  * This file is part of Mini-Auto-Drive.
  *
  * Mini-Auto-Drive is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * Mini-Auto-Drive is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with Mini-Auto-Drive.  If not, see <http://www.gnu.org/licenses/>.
  *
  * @file carlocate-tp.h
  */

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER carlocate

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./carlocate-tp.h"

#if !defined(_CARLOCATE_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _CARLOCATE_TP_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    carlocate,
    cp,
    TP_ARGS(
        int, frameIdArg,
        int, carIdArg,
        int, cpIdArg
    ),
    TP_FIELDS(
        ctf_integer(int, frameId, frameIdArg)
        ctf_integer(int, carId, carIdArg)
        ctf_integer(int, cpId, cpIdArg)
    )
)

#endif /* _CARLOCATE_TP_H */

#include <lttng/tracepoint-event.h>
