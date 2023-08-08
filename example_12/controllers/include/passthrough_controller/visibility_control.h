// Copyright (c) 2023, PAL Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PASSTHROUGH_CONTROLLER__VISIBILITY_CONTROL_H_
#define PASSTHROUGH_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PASSTHROUGH_CONTROLLER_EXPORT __attribute__((dllexport))
#define PASSTHROUGH_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define PASSTHROUGH_CONTROLLER_EXPORT __declspec(dllexport)
#define PASSTHROUGH_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef PASSTHROUGH_CONTROLLER_BUILDING_DLL
#define PASSTHROUGH_CONTROLLER_PUBLIC PASSTHROUGH_CONTROLLER_EXPORT
#else
#define PASSTHROUGH_CONTROLLER_PUBLIC PASSTHROUGH_CONTROLLER_IMPORT
#endif
#define PASSTHROUGH_CONTROLLER_PUBLIC_TYPE PASSTHROUGH_CONTROLLER_PUBLIC
#define PASSTHROUGH_CONTROLLER_LOCAL
#else
#define PASSTHROUGH_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define PASSTHROUGH_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define PASSTHROUGH_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define PASSTHROUGH_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define PASSTHROUGH_CONTROLLER_PUBLIC
#define PASSTHROUGH_CONTROLLER_LOCAL
#endif
#define PASSTHROUGH_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // PASSTHROUGH_CONTROLLER__VISIBILITY_CONTROL_H_
