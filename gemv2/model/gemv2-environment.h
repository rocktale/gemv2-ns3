/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Karsten Roscher
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef GEMV2_ENVIRONMENT_H
#define GEMV2_ENVIRONMENT_H

#include <ns3/simple-ref-count.h>

namespace ns3 {

/*!
 * @brief Class to manage the GEMV^2 environment.
 *
 * This will bundle all actors (vehicles, RSUs) and objects
 * (buildings, foliage) in one place.
 */
class Gemv2Environment : public SimpleRefCount<Gemv2Environment>
{
public:
  // TODO
};

}  // namespace ns3

#endif /* GEMV2_ENVIRONMENT_H */
