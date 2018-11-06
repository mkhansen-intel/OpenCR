/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim */

#ifndef DEMO_H_
#define DEMO_H_

#include "Delta.h"

// #define MAX_MOTION_NUM 12
// #define MAX_MOTION_NUM2 45
// #define MAX_MOTION_NUM3 60
// #define MAX_MOTION_NUM4 60


bool suction_on = false;

#define MAX_MOTION_NUM7 140
const float tool_position7[MAX_MOTION_NUM7][4] = {// { x, y, z}
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  

                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                {-0.001f,  -0.070f,  0.015f, 0},  
                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                
                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                {-0.0580f,  0.030f,  0.013f, 0},  
                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                
                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},

                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   

                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   

                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},

                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},  

                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                {-0.0580f,  0.030f,  0.013f, 0},  
                                                {-0.0580f,  0.030f,  0.036f, 0},  

                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  

                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                {-0.001f,  -0.070f,  0.015f, 0},  
                                                {-0.001f,  -0.070f,  0.038f, 0},  

                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   

                                                // For waiting
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   // count 51 
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  



                                                // 1 -> 9  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.038f, 1},  
                                                { 0.015f,  -0.012f,  0.038f, 1},  
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.038f, 0}, 

                                                // 3 -> 8  
                                                {-0.0583f, -0.035f,  0.035f, 0},  
                                                {-0.0583f, -0.035f,  0.012f, 1},  
                                                {-0.0583f, -0.035f,  0.012f, 1},  
                                                {-0.0583f, -0.035f,  0.012f, 1},  
                                                {-0.0583f, -0.035f,  0.035f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  

                                                // 5 -> 7  
 
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.037f, 1},   
                                                { 0.001f,   0.016f,  0.037f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.037f, 0},  

                                                 // 6 -> 10  
                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.039f, 1},  
                                                {-0.0000f,  0.000f,  0.051f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.051f, 0},  // count = 94 

                                                // For waiting
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  // count = 97
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},    

                                                 // 10 -> 6  
                                                {-0.0000f,  0.000f,  0.049f, 0},  
                                                {-0.0000f,  0.000f,  0.031f, 1},  
                                                {-0.0000f,  0.000f,  0.031f, 1},  
                                                {-0.0000f,  0.000f,  0.031f, 1},  
                                                {-0.0000f,  0.000f,  0.049f, 1},  
                                                { 0.057f,   0.032f,  0.050f, 1},  
                                                { 0.057f,   0.032f,  0.017f, 1},  
                                                { 0.057f,   0.032f,  0.017f, 1},  
                                                { 0.057f,   0.032f,  0.017f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},  

                                                // 7 -> 5  
                                                { 0.001f,   0.016f,  0.037f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.037f, 1},  
                                                { 0.001f,   0.065f,  0.037f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},   

                                                // 8 -> 3  
                                                {-0.016f,  -0.012f,  0.035f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 1},  
                                                {-0.0583f, -0.035f,  0.037f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  

                                                // 9 -> 1  
                                                { 0.015f,  -0.012f,  0.038f, 0},  
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.038f, 1}, 
                                                { 0.055f,  -0.035f,  0.038f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},

                                                // For waiting
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0}
                                                };



#define MAX_MOTION_NUM81 200
#define MAX_MOTION_NUM8 90
const float tool_position8[MAX_MOTION_NUM81][4] = {// { x, y, z}
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  

                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                {-0.001f,  -0.070f,  0.015f, 0},  
                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                
                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                {-0.0580f,  0.030f,  0.013f, 0},  
                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                
                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},

                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   

                                                // For waiting
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  

                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.015f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},   

                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},

                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},  

                                                {-0.0580f,  0.030f,  0.036f, 0},  
                                                {-0.0580f,  0.030f,  0.013f, 0},  
                                                {-0.0580f,  0.030f,  0.036f, 0},  

                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  

                                                {-0.001f,  -0.070f,  0.038f, 0},  
                                                {-0.001f,  -0.070f,  0.015f, 0},  
                                                {-0.001f,  -0.070f,  0.038f, 0},  

                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},   

                                                // For waiting
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},   // count 51 
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  



                                                // 1 -> 9  
                                                { 0.055f,  -0.035f,  0.037f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.037f, 1},  
                                                { 0.015f,  -0.012f,  0.038f, 0},  
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.038f, 0}, 


                                                // 3 -> 8  

                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.015f, 1},  
                                                {-0.0583f, -0.035f,  0.015f, 1},  
                                                {-0.0583f, -0.035f,  0.015f, 1},  
                                                {-0.0583f, -0.035f,  0.037f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  

                                                // 5 -> 7  
 
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.015f, 1},  
                                                { 0.001f,   0.065f,  0.015f, 1},  
                                                { 0.001f,   0.065f,  0.015f, 1},  
                                                { 0.001f,   0.065f,  0.037f, 1},   
                                                { 0.001f,   0.016f,  0.033f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.037f, 0},  

                                                 // 6 -> 10  
                                                { 0.057f,   0.032f,  0.037f, 0},  
                                                { 0.057f,   0.032f,  0.015f, 1},  
                                                { 0.057f,   0.032f,  0.015f, 1},  
                                                { 0.057f,   0.032f,  0.015f, 1},  
                                                { 0.057f,   0.032f,  0.037f, 1},  
                                                {-0.0000f,  0.000f,  0.052f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.052f, 0},  // count = 79

                                                // Hitting the balls
                                                { +0.060f, -0.060f,  0.010f, 0},  
                                                { +0.060f, -0.060f,  0.010f, 0},  
                                                { +0.060f, -0.060f,  0.010f, 0},  
                                                { +0.060f, -0.060f,  0.010f, 0},  
                                                { +0.060f, -0.060f,  0.010f, 0},  
                                                { +0.060f, -0.060f,  0.010f, 0},  
                                                { -0.060f, +0.060f,  0.010f, 0},  
                                                { -0.060f, +0.060f,  0.010f, 0},  
                                                { -0.060f, +0.060f,  0.010f, 0},  
                                                { -0.060f, +0.060f,  0.010f, 0},  
                                                { -0.060f, +0.060f,  0.010f, 0},  
                                                { -0.060f, +0.060f,  0.010f, 0}
                                                // {  0.055f, -0.035f,  0.038f, 0},  
                                                // {  0.055f, -0.035f,  0.038f, 0},  // count = 82
                                                // {  0.055f, -0.035f,  0.038f, 0},  
                                                // {  0.055f, -0.035f,  0.038f, 0},  
                                                // {  0.055f, -0.035f,  0.038f, 0},  
                                                // // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // // {-0.0000f,  0.000f,  0.050f, 0},  

                                                //  // 10 -> 6  
                                                // {-0.0000f,  0.000f,  0.051f, 0},  
                                                // {-0.0000f,  0.000f,  0.033f, 1},  
                                                // {-0.0000f,  0.000f,  0.033f, 1},  
                                                // {-0.0000f,  0.000f,  0.033f, 1},  
                                                // {-0.0000f,  0.000f,  0.051f, 1},  
                                                // { 0.057f,   0.032f,  0.050f, 0},  
                                                // { 0.057f,   0.032f,  0.017f, 0},  
                                                // { 0.057f,   0.032f,  0.017f, 0},  
                                                // { 0.057f,   0.032f,  0.017f, 0},  
                                                // { 0.057f,   0.032f,  0.039f, 0},  

                                                // // 7 -> 5  
                                                // { 0.001f,   0.016f,  0.037f, 0},  
                                                // { 0.001f,   0.016f,  0.014f, 1},  
                                                // { 0.001f,   0.016f,  0.014f, 1},  
                                                // { 0.001f,   0.016f,  0.014f, 1},  
                                                // { 0.001f,   0.016f,  0.037f, 0},  
                                                // { 0.001f,   0.065f,  0.037f, 0},  
                                                // { 0.001f,   0.065f,  0.014f, 0},  
                                                // { 0.001f,   0.065f,  0.014f, 0},  
                                                // { 0.001f,   0.065f,  0.014f, 0},  
                                                // { 0.001f,   0.065f,  0.037f, 0},   

                                                // // 8 -> 3  
                                                // {-0.016f,  -0.012f,  0.035f, 0},  
                                                // {-0.016f,  -0.012f,  0.012f, 1},  
                                                // {-0.016f,  -0.012f,  0.012f, 1},  
                                                // {-0.016f,  -0.012f,  0.012f, 1},  
                                                // {-0.016f,  -0.012f,  0.035f, 0},  
                                                // {-0.0583f, -0.035f,  0.037f, 0},  
                                                // {-0.0583f, -0.035f,  0.014f, 0},  
                                                // {-0.0583f, -0.035f,  0.014f, 0},  
                                                // {-0.0583f, -0.035f,  0.014f, 0},  
                                                // {-0.0583f, -0.035f,  0.037f, 0},  

                                                // // 9 -> 1  
                                                // { 0.015f,  -0.012f,  0.038f, 0},  
                                                // { 0.015f,  -0.012f,  0.015f, 1}, 
                                                // { 0.015f,  -0.012f,  0.015f, 1}, 
                                                // { 0.015f,  -0.012f,  0.015f, 1}, 
                                                // { 0.015f,  -0.012f,  0.038f, 1}, 
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.015f, 0},  
                                                // { 0.055f,  -0.035f,  0.015f, 0},  
                                                // { 0.055f,  -0.035f,  0.015f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},

                                                // // For waiting
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0},  
                                                // { 0.055f,  -0.035f,  0.038f, 0}
                                                };



uint8_t motion_cnt = 0;
uint8_t motion_page = 23;
// bool drawing_flag = true;
int drawing_flag = 0;
float start_angular_position = -PI/6.0f;
// const float move_time = 10.0;
float move_time = 5.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.065f;
float motion_no = 1;

bool motion[1] = {false};

void test()
{
  if (delta.moving() || delta.drawing())
  {
    return;
  }
  else
  {
    if (motion_no == 1){
    // if (~drawing_flag){  // why not working....???
    if (drawing_flag == 0){  // why not working....???
      if (motion_cnt == MAX_MOTION_NUM7){
        motion_cnt = 0;
        motion_no ++;
      }

      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(tool_position7[motion_cnt][0], 
                                                  tool_position7[motion_cnt][1], 
                                                  tool_position7[motion_cnt][2]);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    0.0f, 0.0f, 1.0f);

      delta.setPose(TOOL, target_pose, 0.5f);

      if (tool_position7[motion_cnt][3] == 1){
        digitalWrite(RELAY_PIN, HIGH);      //suction on
        // suction_on = true;
      } 
      else{
        digitalWrite(RELAY_PIN, LOW);      //suction off
        // suction_on = false;
      }

      motion_cnt++;

      if (motion_cnt == 48){
        // drawing = true;  // why not wokring...?
        motion_cnt = 0;
        motion_no ++;
      }

      if (motion_cnt == 97){
        // drawing = true;  // why not wokring...?
        drawing_flag = 1;
      }
    }

    else 
    {
      Vector3f temp_current_position;
      temp_current_position(0) = 0.055f;
      temp_current_position(1) = -0.035f;
      temp_current_position(2) = 0.030f;

      if (motion_page == CIRCLEEDGE2)
      {
        delta.drawInit(CIRCLEEDGE2, move_time, p_init_arg);
        delta.setRadiusForDrawing(CIRCLEEDGE2, radius);
        delta.setStartPositionForDrawing(CIRCLEEDGE2, temp_current_position);
        delta.setStartAngularPositionForDrawing(CIRCLEEDGE2, start_angular_position);
        delta.draw(CIRCLEEDGE2);
        motion_page = 22;
      }
      else
      {
        delta.drawInit(CIRCLEEDGE, move_time, p_init_arg);
        delta.setRadiusForDrawing(CIRCLEEDGE, radius);
        delta.setStartPositionForDrawing(CIRCLEEDGE, temp_current_position);
        delta.setStartAngularPositionForDrawing(CIRCLEEDGE, start_angular_position);
        delta.draw(CIRCLEEDGE);
        motion_page++;
        // drawing = false;  // why not working??
        drawing_flag = 0;
      }
    }
    }
    else if (motion_no == 2) {
      if (drawing_flag == 0){  // why not working....???
      if (motion_cnt == MAX_MOTION_NUM8)
        motion_cnt = 0;

      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(tool_position8[motion_cnt][0], 
                                                  tool_position8[motion_cnt][1], 
                                                  tool_position8[motion_cnt][2]);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    0.0f, 0.0f, 1.0f);

      delta.setPose(TOOL, target_pose, 0.1);

      if (tool_position8[motion_cnt][3] == 1){
        digitalWrite(RELAY_PIN, HIGH);      //suction on
        // suction_on = true;
      } 
      else{
        digitalWrite(RELAY_PIN, LOW);      //suction off
        // suction_on = false;
      }

      motion_cnt++;
      
      if (motion_cnt == 90){
        // drawing = true;  // why not wokring...?
        drawing_flag = 1;
      }
    }

    else 
    {
      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(0,0,0);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    0.0f, 0.0f, 1.0f);

      delta.setPose(TOOL, target_pose, 0.11);
    }
    }
  }
}



void test2()
{
  if (delta.moving() || delta.drawing())
  {
    return;
  }
  else
  {
    if (drawing_flag == 0){  // why not working....???
      if (motion_cnt == MAX_MOTION_NUM8)
        motion_cnt = 0;

      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(tool_position8[motion_cnt][0], 
                                                  tool_position8[motion_cnt][1], 
                                                  tool_position8[motion_cnt][2]);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    0.0f, 0.0f, 1.0f);

      delta.setPose(TOOL, target_pose, 0.10);

      if (tool_position8[motion_cnt][3] == 1){
        digitalWrite(RELAY_PIN, HIGH);      //suction on
        // suction_on = true;
      } 
      else{
        digitalWrite(RELAY_PIN, LOW);      //suction off
        // suction_on = false;
      }

      motion_cnt++;
      
      if (motion_cnt == 90){
        // drawing = true;  // why not wokring...?
        drawing_flag = 1;
      }
    }

    else 
    {
      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(0,0,0);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    0.0f, 0.0f, 1.0f);

      delta.setPose(TOOL, target_pose, 0.11);
    }
  }
}




void test3()
{
  if (delta.moving() || delta.drawing())
  {
    return;
  }
  else
  {
    Vector3f temp_current_position;
    temp_current_position(0) = 0.055f;
    temp_current_position(1) = -0.035f;
    temp_current_position(2) = 0.020f;

    if (motion_page == CIRCLEEDGE)
    {
      delta.drawInit(CIRCLEEDGE, move_time, p_init_arg);
      delta.setRadiusForDrawing(CIRCLEEDGE, radius);
      delta.setStartPositionForDrawing(CIRCLEEDGE, temp_current_position);
      delta.setStartAngularPositionForDrawing(CIRCLEEDGE, start_angular_position);
      delta.draw(CIRCLEEDGE);
      motion_page++;
    }
    else
    {
      delta.drawInit(CIRCLEEDGE2, move_time, p_init_arg);
      delta.setRadiusForDrawing(CIRCLEEDGE2, radius);
      delta.setStartPositionForDrawing(CIRCLEEDGE2, temp_current_position);
      delta.setStartAngularPositionForDrawing(CIRCLEEDGE2, start_angular_position);
      delta.draw(CIRCLEEDGE2);
      motion_page = 22;
    }
  }
}
#define MAX_MOTION_NUM 10
const float tool_position[MAX_MOTION_NUM][4] = {// { x, y, z}
                                                { 0.000f,  0.060f, 0.000f, 0},  
                                                { 0.000f,  0.000f, 0.000f, 0},  
                                                { 0.000f, -0.060f, 0.000f, 0},  
                                                { 0.000f,  0.000f, 0.000f, 0},  
                                                { 0.060f,  0.000f, 0.000f, 0},  
                                                { 0.000f,  0.000f, 0.000f, 0},  
                                                {-0.060f,  0.000f, 0.000f, 0},  
                                                {-0.000f,  0.000f, 0.000f, 0},  
                                                { 0.000f,  0.000f, 0.030f, 0},  
                                                {-0.000f,  0.000f, 0.000f, 0}  
                                                };


void test4()
{
  if (delta.moving() || delta.drawing())
  {
    return;
  }
  else
  {
    if (drawing_flag == 0){  // why not working....???

      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(tool_position[motion_cnt][0], 
                                                  tool_position[motion_cnt][1], 
                                                  tool_position[motion_cnt][2]);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    0.0f, 0.0f, 1.0f);

      delta.setPose(TOOL, target_pose, 1.0);

      motion_cnt++;
      
      if (motion_cnt == 90){
        // drawing = true;  // why not wokring...?
        drawing_flag = 1;
      }
    }

    else 
    {
      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(0,0,0);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    0.0f, 0.0f, 1.0f);

      delta.setPose(TOOL, target_pose, 0.11);
    }
  }
}





#endif // DEMO_H_















// const float tool_position[MAX_MOTION_NUM][3] = {// { x, y, z}
//                                                 {-0.060f,  0.000f, 0.030f},  
//                                                 {-0.060f,  0.000f, 0.020f},  
//                                                 {-0.060f,  0.000f, 0.030f},  
//                                                 { 0.000f,  0.060f, 0.030f},  
//                                                 { 0.000f,  0.060f, 0.020f},  
//                                                 { 0.000f,  0.060f, 0.030f},  
//                                                 { 0.060f,  0.000f, 0.030f},  
//                                                 { 0.060f,  0.000f, 0.020f},  
//                                                 { 0.060f,  0.000f, 0.030f},  
//                                                 {-0.000f, -0.060f, 0.030f},  
//                                                 {-0.000f, -0.060f, 0.020f},  
//                                                 {-0.000f, -0.060f, 0.030f}
//                                                 };
// const float tool_position2[MAX_MOTION_NUM2][4] = {// { x, y, z}
//                                                 // { 0.0563f, -0.0325f, 0.030f},  
//                                                 // { 0.0563f, -0.0325f, 0.014f},  
//                                                 // { 0.0563f, -0.0325f, 0.030f},  
//                                                 { 0.055f,  -0.035f,  0.038f, 0},  
//                                                 { 0.055f,  -0.035f,  0.015f, 1},  
//                                                 { 0.055f,  -0.035f,  0.015f, 1},  
//                                                 { 0.055f,  -0.035f,  0.015f, 1},  
//                                                 { 0.055f,  -0.035f,  0.038f, 1},  
//                                                 // { 0.000f,  -0.065f,  0.030f},  
//                                                 // { 0.000f,  -0.065f,  0.014f},  
//                                                 // { 0.000f,  -0.065f,  0.030f},  
//                                                 {-0.001f,  -0.070f,  0.038f, 1},  
//                                                 {-0.001f,  -0.070f,  0.015f, 1},  
//                                                 {-0.001f,  -0.070f,  0.015f, 0},  
//                                                 {-0.001f,  -0.070f,  0.015f, 0},  
//                                                 {-0.001f,  -0.070f,  0.038f, 0},  
//                                                 // {-0.0563f, -0.0325f, 0.030f},  
//                                                 // {-0.0563f, -0.0325f, 0.014f},  
//                                                 // {-0.0563f, -0.0325f, 0.030f},  
//                                                 {-0.0583f, -0.035f,  0.037f, 0},  
//                                                 {-0.0583f, -0.035f,  0.014f, 1},  
//                                                 {-0.0583f, -0.035f,  0.014f, 1},  
//                                                 {-0.0583f, -0.035f,  0.014f, 1},  
//                                                 {-0.0583f, -0.035f,  0.037f, 1},  
//                                                 // {-0.0563f,  0.0325f, 0.030f},  
//                                                 // {-0.0563f,  0.0325f, 0.014f},  
//                                                 // {-0.0563f,  0.0325f, 0.030f},  
//                                                 {-0.0580f,  0.030f,  0.036f, 1},  
//                                                 {-0.0580f,  0.030f,  0.013f, 1},  
//                                                 {-0.0580f,  0.030f,  0.013f, 0},  
//                                                 {-0.0580f,  0.030f,  0.013f, 0},  
//                                                 {-0.0580f,  0.030f,  0.036f, 0},  
//                                                 // { 0.000f,   0.065f,  0.030f},  
//                                                 // { 0.000f,   0.065f,  0.014f},  
//                                                 // { 0.000f,   0.065f,  0.030f},  
//                                                 { 0.001f,   0.065f,  0.037f, 0},  
//                                                 { 0.001f,   0.065f,  0.014f, 1},  
//                                                 { 0.001f,   0.065f,  0.014f, 1},  
//                                                 { 0.001f,   0.065f,  0.014f, 1},  
//                                                 { 0.001f,   0.065f,  0.037f, 1},  
//                                                 // { 0.0563f,  0.0325f, 0.030f},  
//                                                 // { 0.0563f,  0.0325f, 0.014f},  
//                                                 // { 0.0563f,  0.0325f, 0.030f},  
//                                                 { 0.057f,   0.032f,  0.039f, 1},  
//                                                 { 0.057f,   0.032f,  0.016f, 0},  
//                                                 { 0.057f,   0.032f,  0.016f, 0},  
//                                                 { 0.057f,   0.032f,  0.016f, 0},  
//                                                 { 0.057f,   0.032f,  0.039f, 0},  
//                                                 // { 0.000f,   0.0173f, 0.030f},  
//                                                 // { 0.000f,   0.0173f, 0.014f},  
//                                                 // { 0.000f,   0.0173f, 0.030f},  
//                                                 { 0.001f,   0.016f,  0.037f, 0},  
//                                                 { 0.001f,   0.016f,  0.014f, 0},  
//                                                 { 0.001f,   0.016f,  0.014f, 0},  
//                                                 { 0.001f,   0.016f,  0.014f, 0},  
//                                                 { 0.001f,   0.016f,  0.037f, 0},  
//                                                 // {-0.0150f, -0.00865f,0.030f},  
//                                                 // {-0.0150f, -0.00865f,0.014f},  
//                                                 // {-0.0150f, -0.00865f,0.030f},  
//                                                 {-0.016f,  -0.012f,  0.035f, 0},  
//                                                 {-0.016f,  -0.012f,  0.012f, 0},  
//                                                 {-0.016f,  -0.012f,  0.012f, 0},  
//                                                 {-0.016f,  -0.012f,  0.012f, 0},  
//                                                 {-0.016f,  -0.012f,  0.035f, 0},  
//                                                 // { 0.0150f, -0.00865f,0.030f},  
//                                                 // { 0.0150f, -0.00865f,0.014f},  
//                                                 // { 0.0150f, -0.00865f,0.030f}
//                                                 { 0.015f,  -0.012f,  0.038f, 0},  
//                                                 { 0.015f,  -0.012f,  0.015f, 0}, 
//                                                 { 0.015f,  -0.012f,  0.015f, 0}, 
//                                                 { 0.015f,  -0.012f,  0.015f, 0}, 
//                                                 { 0.015f,  -0.012f,  0.038f, 0} 
//                                                 };



// const float tool_position3[MAX_MOTION_NUM3][4] = {// { x, y, z}
//                                                 // 1 -> 3  
//                                                 { 0.056f,  -0.037f,  0.039f, 0},  
//                                                 { 0.056f,  -0.037f,  0.016f, 1},  
//                                                 { 0.056f,  -0.037f,  0.016f, 1},  
//                                                 { 0.056f,  -0.037f,  0.016f, 1},  
//                                                 { 0.056f,  -0.037f,  0.039f, 1},  
//                                                 {-0.0563f, -0.036f,  0.037f, 1},  
//                                                 {-0.0563f, -0.036f,  0.014f, 1},  
//                                                 {-0.0563f, -0.036f,  0.014f, 0},  
//                                                 {-0.0563f, -0.036f,  0.014f, 0},  
//                                                 {-0.0563f, -0.036f,  0.037f, 0},  

//                                                 // 4 -> 6  
//                                                 {-0.0560f,  0.028f,  0.036f, 0},  
//                                                 {-0.0560f,  0.028f,  0.013f, 1},  
//                                                 {-0.0560f,  0.028f,  0.013f, 1},  
//                                                 {-0.0560f,  0.028f,  0.013f, 1},  
//                                                 {-0.0560f,  0.028f,  0.036f, 1},  
//                                                 { 0.059f,   0.030f,  0.039f, 1},  
//                                                 { 0.059f,   0.030f,  0.016f, 1},  
//                                                 { 0.059f,   0.030f,  0.016f, 0},  
//                                                 { 0.059f,   0.030f,  0.016f, 0},  
//                                                 { 0.059f,   0.030f,  0.039f, 0},  

//                                                 // 5 -> 1  
//                                                 { 0.003f,   0.063f,  0.038f, 0},  
//                                                 { 0.003f,   0.063f,  0.015f, 1},  
//                                                 { 0.003f,   0.063f,  0.015f, 1},  
//                                                 { 0.003f,   0.063f,  0.015f, 1},  
//                                                 { 0.003f,   0.063f,  0.038f, 1},  
//                                                 { 0.056f,  -0.037f,  0.039f, 1},  
//                                                 { 0.056f,  -0.037f,  0.016f, 1},  
//                                                 { 0.056f,  -0.037f,  0.016f, 0},  
//                                                 { 0.056f,  -0.037f,  0.016f, 0},  
//                                                 { 0.056f,  -0.037f,  0.039f, 0},  

//                                                 // 2 -> 4  
//                                                 { 0.000f,  -0.071f,  0.040f, 0},  
//                                                 { 0.000f,  -0.071f,  0.017f, 1},  
//                                                 { 0.000f,  -0.071f,  0.017f, 1},  
//                                                 { 0.000f,  -0.071f,  0.017f, 1},  
//                                                 { 0.000f,  -0.071f,  0.040f, 1},  
//                                                 {-0.0560f,  0.028f,  0.036f, 1},  
//                                                 {-0.0560f,  0.028f,  0.013f, 1},  
//                                                 {-0.0560f,  0.028f,  0.013f, 0},  
//                                                 {-0.0560f,  0.028f,  0.013f, 0},  
//                                                 {-0.0560f,  0.028f,  0.036f, 0},  

//                                                 // 3 -> 5  
//                                                 {-0.0563f, -0.036f,  0.037f, 0},  
//                                                 {-0.0563f, -0.036f,  0.014f, 1},  
//                                                 {-0.0563f, -0.036f,  0.014f, 1},  
//                                                 {-0.0563f, -0.036f,  0.014f, 1},  
//                                                 {-0.0563f, -0.036f,  0.037f, 1},  
//                                                 { 0.003f,   0.063f,  0.038f, 1},  
//                                                 { 0.003f,   0.063f,  0.015f, 1},  
//                                                 { 0.003f,   0.063f,  0.015f, 0},  
//                                                 { 0.003f,   0.063f,  0.015f, 0},  
//                                                 { 0.003f,   0.063f,  0.038f, 0},  

//                                                 // 6 -> 2 
//                                                 { 0.059f,   0.030f,  0.039f, 0},  
//                                                 { 0.059f,   0.030f,  0.016f, 1},  
//                                                 { 0.059f,   0.030f,  0.016f, 1},  
//                                                 { 0.059f,   0.030f,  0.016f, 1},  
//                                                 { 0.059f,   0.030f,  0.039f, 1}, 
//                                                 { 0.000f,  -0.071f,  0.040f, 1},  
//                                                 { 0.000f,  -0.071f,  0.017f, 1},  
//                                                 { 0.000f,  -0.071f,  0.017f, 0},  
//                                                 { 0.000f,  -0.071f,  0.017f, 0},  
//                                                 { 0.000f,  -0.071f,  0.040f, 0}  
//                                                 };
                                                


// // const float tool_position4[MAX_MOTION_NUM4][4] = {// { x, y, z}
// //                                                 // 1 -> 3  
// //                                                 { 0.056f,  -0.037f,  0.049f, 0},  
// //                                                 { 0.056f,  -0.037f,  0.031f, 1},  
// //                                                 { 0.056f,  -0.037f,  0.031f, 1},  
// //                                                 { 0.056f,  -0.037f,  0.031f, 1},  
// //                                                 { 0.056f,  -0.037f,  0.049f, 1},  
// //                                                 {-0.0563f, -0.036f,  0.047f, 1},  
// //                                                 {-0.0563f, -0.036f,  0.029f, 1},  
// //                                                 {-0.0563f, -0.036f,  0.029f, 0},  
// //                                                 {-0.0563f, -0.036f,  0.029f, 0},  
// //                                                 {-0.0563f, -0.036f,  0.047f, 0},  

// //                                                 // 4 -> 6  
// //                                                 {-0.0560f,  0.028f,  0.046f, 0},  
// //                                                 {-0.0560f,  0.028f,  0.028f, 1},  
// //                                                 {-0.0560f,  0.028f,  0.028f, 1},  
// //                                                 {-0.0560f,  0.028f,  0.028f, 1},  
// //                                                 {-0.0560f,  0.028f,  0.046f, 1},  
// //                                                 { 0.059f,   0.030f,  0.049f, 1},  
// //                                                 { 0.059f,   0.030f,  0.031f, 1},  
// //                                                 { 0.059f,   0.030f,  0.031f, 0},  
// //                                                 { 0.059f,   0.030f,  0.031f, 0},  
// //                                                 { 0.059f,   0.030f,  0.049f, 0},  

// //                                                 // 5 -> 1  
// //                                                 { 0.003f,   0.063f,  0.048f, 0},  
// //                                                 { 0.003f,   0.063f,  0.030f, 1},  
// //                                                 { 0.003f,   0.063f,  0.030f, 1},  
// //                                                 { 0.003f,   0.063f,  0.030f, 1},  
// //                                                 { 0.003f,   0.063f,  0.048f, 1},  
// //                                                 { 0.056f,  -0.037f,  0.049f, 1},  
// //                                                 { 0.056f,  -0.037f,  0.031f, 1},  
// //                                                 { 0.056f,  -0.037f,  0.031f, 0},  
// //                                                 { 0.056f,  -0.037f,  0.031f, 0},  
// //                                                 { 0.056f,  -0.037f,  0.049f, 0},  

// //                                                 // 2 -> 4  
// //                                                 { 0.000f,  -0.071f,  0.050f, 0},  
// //                                                 { 0.000f,  -0.071f,  0.032f, 1},  
// //                                                 { 0.000f,  -0.071f,  0.032f, 1},  
// //                                                 { 0.000f,  -0.071f,  0.032f, 1},  
// //                                                 { 0.000f,  -0.071f,  0.050f, 1},  
// //                                                 {-0.0560f,  0.028f,  0.046f, 1},  
// //                                                 {-0.0560f,  0.028f,  0.028f, 1},  
// //                                                 {-0.0560f,  0.028f,  0.028f, 0},  
// //                                                 {-0.0560f,  0.028f,  0.028f, 0},  
// //                                                 {-0.0560f,  0.028f,  0.046f, 0},  

// //                                                 // 3 -> 5  
// //                                                 {-0.0563f, -0.036f,  0.047f, 0},  
// //                                                 {-0.0563f, -0.036f,  0.029f, 1},  
// //                                                 {-0.0563f, -0.036f,  0.029f, 1},  
// //                                                 {-0.0563f, -0.036f,  0.029f, 1},  
// //                                                 {-0.0563f, -0.036f,  0.047f, 1},  
// //                                                 { 0.003f,   0.063f,  0.048f, 1},  
// //                                                 { 0.003f,   0.063f,  0.030f, 1},  
// //                                                 { 0.003f,   0.063f,  0.030f, 0},  
// //                                                 { 0.003f,   0.063f,  0.030f, 0},  
// //                                                 { 0.003f,   0.063f,  0.048f, 0},  

// //                                                 // 6 -> 2 
// //                                                 { 0.059f,   0.030f,  0.049f, 0},  
// //                                                 { 0.059f,   0.030f,  0.031f, 1},  
// //                                                 { 0.059f,   0.030f,  0.031f, 1},  
// //                                                 { 0.059f,   0.030f,  0.031f, 1},  
// //                                                 { 0.059f,   0.030f,  0.049f, 1}, 
// //                                                 { 0.000f,  -0.071f,  0.050f, 1},  
// //                                                 { 0.000f,  -0.071f,  0.032f, 1},  
// //                                                 { 0.000f,  -0.071f,  0.032f, 0},  
// //                                                 { 0.000f,  -0.071f,  0.032f, 0},  
// //                                                 { 0.000f,  -0.071f,  0.050f, 0}  
// //                                                 };
                                                

// #define MAX_MOTION_NUM5 80
// const float tool_position5[MAX_MOTION_NUM5][4] = {// { x, y, z}
//                                                 // 1 -> 9  
//                                                 { 0.055f,  -0.035f,  0.038f, 0},  
//                                                 { 0.055f,  -0.035f,  0.015f, 1},  
//                                                 { 0.055f,  -0.035f,  0.015f, 1},  
//                                                 { 0.055f,  -0.035f,  0.015f, 1},  
//                                                 { 0.055f,  -0.035f,  0.038f, 1},  
//                                                 { 0.015f,  -0.012f,  0.038f, 1},  
//                                                 { 0.015f,  -0.012f,  0.015f, 0}, 
//                                                 { 0.015f,  -0.012f,  0.015f, 0}, 
//                                                 { 0.015f,  -0.012f,  0.015f, 0}, 
//                                                 { 0.015f,  -0.012f,  0.038f, 0}, 

//                                                 // 3 -> 8  
//                                                 {-0.0583f, -0.035f,  0.037f, 0},  
//                                                 {-0.0583f, -0.035f,  0.014f, 1},  
//                                                 {-0.0583f, -0.035f,  0.014f, 1},  
//                                                 {-0.0583f, -0.035f,  0.014f, 1},  
//                                                 {-0.0583f, -0.035f,  0.037f, 1},  
//                                                 {-0.016f,  -0.012f,  0.035f, 1},  
//                                                 {-0.016f,  -0.012f,  0.012f, 0},  
//                                                 {-0.016f,  -0.012f,  0.012f, 0},  
//                                                 {-0.016f,  -0.012f,  0.012f, 0},  
//                                                 {-0.016f,  -0.012f,  0.035f, 0},  

//                                                 // 5 -> 7  
 
//                                                 { 0.001f,   0.065f,  0.037f, 0},  
//                                                 { 0.001f,   0.065f,  0.014f, 1},  
//                                                 { 0.001f,   0.065f,  0.014f, 1},  
//                                                 { 0.001f,   0.065f,  0.014f, 1},  
//                                                 { 0.001f,   0.065f,  0.037f, 1},   
//                                                 { 0.001f,   0.016f,  0.037f, 1},  
//                                                 { 0.001f,   0.016f,  0.014f, 0},  
//                                                 { 0.001f,   0.016f,  0.014f, 0},  
//                                                 { 0.001f,   0.016f,  0.014f, 0},  
//                                                 { 0.001f,   0.016f,  0.037f, 0},  

//                                                  // 6 -> 10  
//                                                 { 0.057f,   0.032f,  0.039f, 0},  
//                                                 { 0.057f,   0.032f,  0.016f, 1},  
//                                                 { 0.057f,   0.032f,  0.016f, 1},  
//                                                 { 0.057f,   0.032f,  0.016f, 1},  
//                                                 { 0.057f,   0.032f,  0.039f, 1},  
//                                                 {-0.0000f,  0.000f,  0.050f, 1},  
//                                                 {-0.0000f,  0.000f,  0.032f, 0},  
//                                                 {-0.0000f,  0.000f,  0.032f, 0},  
//                                                 {-0.0000f,  0.000f,  0.032f, 0},  
//                                                 {-0.0000f,  0.000f,  0.050f, 0},  

//                                                  // 10 -> 6  
//                                                 {-0.0000f,  0.000f,  0.050f, 0},  
//                                                 {-0.0000f,  0.000f,  0.032f, 0},  
//                                                 {-0.0000f,  0.000f,  0.032f, 1},  
//                                                 {-0.0000f,  0.000f,  0.032f, 1},  
//                                                 {-0.0000f,  0.000f,  0.050f, 1},  
//                                                 { 0.057f,   0.032f,  0.039f, 1},  
//                                                 { 0.057f,   0.032f,  0.017f, 0},  
//                                                 { 0.057f,   0.032f,  0.017f, 0},  
//                                                 { 0.057f,   0.032f,  0.017f, 0},  
//                                                 { 0.057f,   0.032f,  0.040f, 0},  

//                                                 // 7 -> 5  
//                                                 { 0.001f,   0.016f,  0.037f, 0},  
//                                                 { 0.001f,   0.016f,  0.014f, 0},  
//                                                 { 0.001f,   0.016f,  0.014f, 1},  
//                                                 { 0.001f,   0.016f,  0.014f, 1},  
//                                                 { 0.001f,   0.016f,  0.037f, 1},  
//                                                 { 0.001f,   0.065f,  0.037f, 1},  
//                                                 { 0.001f,   0.065f,  0.014f, 0},  
//                                                 { 0.001f,   0.065f,  0.014f, 0},  
//                                                 { 0.001f,   0.065f,  0.014f, 0},  
//                                                 { 0.001f,   0.065f,  0.037f, 0},   

//                                                 // 8 -> 3  
//                                                 {-0.016f,  -0.012f,  0.035f, 0},  
//                                                 {-0.016f,  -0.012f,  0.012f, 0},  
//                                                 {-0.016f,  -0.012f,  0.012f, 1},  
//                                                 {-0.016f,  -0.012f,  0.012f, 1},  
//                                                 {-0.016f,  -0.012f,  0.035f, 1},  
//                                                 {-0.0583f, -0.035f,  0.037f, 1},  
//                                                 {-0.0583f, -0.035f,  0.014f, 0},  
//                                                 {-0.0583f, -0.035f,  0.014f, 0},  
//                                                 {-0.0583f, -0.035f,  0.014f, 0},  
//                                                 {-0.0583f, -0.035f,  0.037f, 0},  

//                                                 // 9 -> 1  
//                                                 { 0.015f,  -0.012f,  0.038f, 0},  
//                                                 { 0.015f,  -0.012f,  0.015f, 0}, 
//                                                 { 0.015f,  -0.012f,  0.015f, 1}, 
//                                                 { 0.015f,  -0.012f,  0.015f, 1}, 
//                                                 { 0.015f,  -0.012f,  0.038f, 1}, 
//                                                 { 0.055f,  -0.035f,  0.038f, 1},  
//                                                 { 0.055f,  -0.035f,  0.015f, 0},  
//                                                 { 0.055f,  -0.035f,  0.015f, 0},  
//                                                 { 0.055f,  -0.035f,  0.015f, 0},  
//                                                 { 0.055f,  -0.035f,  0.038f, 0}
//                                                 };
                                                

// #define MAX_MOTION_NUM6 36
// const float tool_position6[MAX_MOTION_NUM6][4] = {// { x, y, z}
//                                                 { 0.055f,  -0.035f,  0.038f, 1},  
//                                                 { 0.055f,  -0.035f,  0.015f, 1},  
//                                                 { 0.055f,  -0.035f,  0.038f, 1},  

//                                                 {-0.001f,  -0.070f,  0.038f, 1},  
//                                                 {-0.001f,  -0.070f,  0.015f, 0},  
//                                                 {-0.001f,  -0.070f,  0.038f, 0},  
                                                
//                                                 {-0.0583f, -0.035f,  0.037f, 0},  
//                                                 {-0.0583f, -0.035f,  0.014f, 1},  
//                                                 {-0.0583f, -0.035f,  0.037f, 1},  
                                                
//                                                 {-0.0580f,  0.030f,  0.036f, 1},  
//                                                 {-0.0580f,  0.030f,  0.013f, 0},  
//                                                 {-0.0580f,  0.030f,  0.036f, 0},  
                                                
//                                                 { 0.001f,   0.065f,  0.037f, 0},  
//                                                 { 0.001f,   0.065f,  0.014f, 1},  
//                                                 { 0.001f,   0.065f,  0.037f, 1},  
                                                
//                                                 { 0.057f,   0.032f,  0.039f, 1},  
//                                                 { 0.057f,   0.032f,  0.016f, 0},  
//                                                 { 0.057f,   0.032f,  0.039f, 0},
                                                
//                                                 { 0.055f,  -0.035f,  0.038f, 1},  
//                                                 { 0.055f,  -0.035f,  0.015f, 1},  
//                                                 { 0.055f,  -0.035f,  0.038f, 1},   

//                                                 { 0.057f,   0.032f,  0.039f, 1},  
//                                                 { 0.057f,   0.032f,  0.016f, 0},  
//                                                 { 0.057f,   0.032f,  0.039f, 0},

//                                                 { 0.001f,   0.065f,  0.037f, 0},  
//                                                 { 0.001f,   0.065f,  0.014f, 1},  
//                                                 { 0.001f,   0.065f,  0.037f, 1},  

//                                                 {-0.0580f,  0.030f,  0.036f, 1},  
//                                                 {-0.0580f,  0.030f,  0.013f, 0},  
//                                                 {-0.0580f,  0.030f,  0.036f, 0},  

//                                                 {-0.0583f, -0.035f,  0.037f, 0},  
//                                                 {-0.0583f, -0.035f,  0.014f, 1},  
//                                                 {-0.0583f, -0.035f,  0.037f, 1},  

                                                // {-0.001f,  -0.070f,  0.038f, 1},  
                                                // {-0.001f,  -0.070f,  0.015f, 0},  
                                                // {-0.001f,  -0.070f,  0.038f, 0}  
                                                // };



