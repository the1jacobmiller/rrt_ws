/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <queue>
#include <vector>
#include <iostream>

//#include "cyber/common/log.h"

//
// void CHECK_TEST() {
//   return;
// }
//
// void CHECK_EQ(int a, int b){
//   if (a != b) {
//     std::cout<<"CHECK_EQ failed";
//     exit(0);
//   }
//
// }
//



namespace apollo {
namespace perception {
namespace common {

void check_eq_mod(int a, int b);

template <typename T>
void check_notnull_mod(T* listToCheck) {
  if (listToCheck == nullptr) { std::cout<<"CHECK FAILED!!!"<<std::endl; exit(0); }
}


void check_mod(bool b);


void sometestFunction(int meow);



/*
* @brief: bfs based connected component analysis
* @params[IN] graph: input graph for connected component analysis
* @params[OUT] components: connected components of input graph
* @return nothing
* */
void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components);

}  // namespace common
}  // namespace perception
}  // namespace apollo
