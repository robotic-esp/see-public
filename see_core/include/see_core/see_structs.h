// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef SEE_STRUCTS_H
#define SEE_STRUCTS_H

#include <string>
#include <vector>

#include <see_common/common_structs.h>

//! SEE(++) Parameters
/*! Parameters for SEE(++) */
struct SeeParams : AlgParams {
  int tau;   /*!< Maximum number of frontiers to update for a single view */
  float psi; /*!< The search radius used for detecting occlusions */
  float ups; /*!< The search radius used for evaluating visibility */
};

#endif // SEE_STRUCTS_H
