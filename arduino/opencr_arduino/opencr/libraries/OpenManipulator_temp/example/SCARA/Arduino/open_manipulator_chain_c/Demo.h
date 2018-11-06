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

#include "Chain.h"

#define MAX_MOTION_NUM 12
#define MAX_MOTION_NUM2 45
#define MAX_MOTION_NUM3 60
#define MAX_MOTION_NUM4 60


bool suction_on = false;


#define MAX_MOTION_NUM7 146
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

                                                // For waiting
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.038f, 0},  
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
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.038f, 0}, 

                                                // 3 -> 8  
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.037f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  

                                                // 5 -> 7  
 
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.037f, 1},   
                                                { 0.001f,   0.016f,  0.037f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.037f, 0},  

                                                 // 6 -> 10  
                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.039f, 1},  
                                                {-0.0000f,  0.000f,  0.051f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.051f, 0},  // count = 94 

                                                // For waiting
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  // count = 97
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  

                                                 // 10 -> 6  
                                                {-0.0000f,  0.000f,  0.051f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.051f, 1},  
                                                { 0.057f,   0.032f,  0.050f, 0},  
                                                { 0.057f,   0.032f,  0.017f, 0},  
                                                { 0.057f,   0.032f,  0.017f, 0},  
                                                { 0.057f,   0.032f,  0.017f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},  

                                                // 7 -> 5  
                                                { 0.001f,   0.016f,  0.037f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.037f, 1},  
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},   

                                                // 8 -> 3  
                                                {-0.016f,  -0.012f,  0.035f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 1},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  

                                                // 9 -> 1  
                                                { 0.015f,  -0.012f,  0.038f, 0},  
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.038f, 1}, 
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
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



#define MAX_MOTION_NUM8 131
const float tool_position8[MAX_MOTION_NUM8][4] = {// { x, y, z}
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
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.015f, 1},  
                                                { 0.055f,  -0.035f,  0.038f, 1},  
                                                { 0.015f,  -0.012f,  0.038f, 1},  
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.015f, 0}, 
                                                { 0.015f,  -0.012f,  0.038f, 0}, 

                                                // 3 -> 8  
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.014f, 1},  
                                                {-0.0583f, -0.035f,  0.037f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 0},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  

                                                // 5 -> 7  
 
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.014f, 1},  
                                                { 0.001f,   0.065f,  0.037f, 1},   
                                                { 0.001f,   0.016f,  0.037f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 0},  
                                                { 0.001f,   0.016f,  0.037f, 0},  

                                                 // 6 -> 10  
                                                { 0.057f,   0.032f,  0.039f, 0},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.016f, 1},  
                                                { 0.057f,   0.032f,  0.039f, 1},  
                                                {-0.0000f,  0.000f,  0.051f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 0},  
                                                {-0.0000f,  0.000f,  0.051f, 0},  // count = 94 

                                                // For waiting
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  // count = 82
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                {  0.055f, -0.035f,  0.038f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  
                                                // {-0.0000f,  0.000f,  0.050f, 0},  

                                                 // 10 -> 6  
                                                {-0.0000f,  0.000f,  0.051f, 0},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.033f, 1},  
                                                {-0.0000f,  0.000f,  0.051f, 1},  
                                                { 0.057f,   0.032f,  0.050f, 0},  
                                                { 0.057f,   0.032f,  0.017f, 0},  
                                                { 0.057f,   0.032f,  0.017f, 0},  
                                                { 0.057f,   0.032f,  0.017f, 0},  
                                                { 0.057f,   0.032f,  0.039f, 0},  

                                                // 7 -> 5  
                                                { 0.001f,   0.016f,  0.037f, 0},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.014f, 1},  
                                                { 0.001f,   0.016f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.014f, 0},  
                                                { 0.001f,   0.065f,  0.037f, 0},   

                                                // 8 -> 3  
                                                {-0.016f,  -0.012f,  0.035f, 0},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.012f, 1},  
                                                {-0.016f,  -0.012f,  0.035f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.014f, 0},  
                                                {-0.0583f, -0.035f,  0.037f, 0},  

                                                // 9 -> 1  
                                                { 0.015f,  -0.012f,  0.038f, 0},  
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.015f, 1}, 
                                                { 0.015f,  -0.012f,  0.038f, 1}, 
                                                { 0.055f,  -0.035f,  0.038f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
                                                { 0.055f,  -0.035f,  0.015f, 0},  
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



uint8_t motion_cnt2 = 0;
uint8_t motion_page = 23;
// bool drawing_flag = true;
int drawing_flag = 0;
float start_angular_position = -PI/6.0f;
// const float move_time = 10.0f;
float move_time = 5.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.025f;

// bool motion[1] = {false};

void test()
{
  if (chain.moving() || chain.drawing())
  {
    return;
  }
  else
  {
    // if (~drawing_flag){  // why not working....???
    if (drawing_flag == 0){  // why not working....???
      if (motion_cnt2 == MAX_MOTION_NUM7)
        motion_cnt2 = 0;

      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(tool_position7[motion_cnt2][0]-0.060, 
                                                  tool_position7[motion_cnt2][1], 
                                                  tool_position7[motion_cnt2][2]+0.060);
      // target_pose.orientation = OM_MATH::makeMatrix3(0.0f, 0.0f, 0.0f,
      //                                               0.0f, 1.0f, 0.0f,
      //                                               -1.0f, 0.0f, 0.0f);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    -0.0f, 0.0f, 1.0f);

      chain.setPose(TOOL, target_pose, 0.50);

      if (tool_position7[motion_cnt2][3] == 1){
        // digitalWrite(RELAY_PIN, HIGH);      //suction on
        // suction_on = true;
      } 
      else{
        // digitalWrite(RELAY_PIN, LOW);      //suction off
        // suction_on = false;
      }

      motion_cnt2++;
      
      if (motion_cnt2 == 97){
        // drawing = true;  // why not wokring...?
        drawing_flag = 1;
      }
    }
    else 
    {
      Vector3f temp_current_position;
      temp_current_position(0) = 0.000f;
      temp_current_position(1) = -0.035f;
      temp_current_position(2) = 0.100f;

      if (motion_page == CIRCLEEDGE2)
      {
        chain.drawInit(CIRCLEEDGE2, move_time, p_init_arg);
        chain.setRadiusForDrawing(CIRCLEEDGE2, radius);
        chain.setStartPositionForDrawing(CIRCLEEDGE2, temp_current_position);
        chain.setStartAngularPositionForDrawing(CIRCLEEDGE2, start_angular_position);
        chain.draw(CIRCLEEDGE2);
        motion_page = 22;
      }
      else
      {
        chain.drawInit(CIRCLEEDGE, move_time, p_init_arg);
        chain.setRadiusForDrawing(CIRCLEEDGE, radius);
        chain.setStartPositionForDrawing(CIRCLEEDGE, temp_current_position);
        chain.setStartAngularPositionForDrawing(CIRCLEEDGE, start_angular_position);
        chain.draw(CIRCLEEDGE);
        motion_page++;
        // drawing = false;  // why not working??
        drawing_flag = 0;
      }
    }
  }
}



void test2()
{
  if (chain.moving() || chain.drawing())
  {
    return;
  }
  else
  {
    // if (~drawing_flag){  // why not working....???
    if (drawing_flag == 0){  // why not working....???
      if (motion_cnt2 == MAX_MOTION_NUM8)
        motion_cnt2 = 0;

      Pose target_pose;
      target_pose.position = OM_MATH::makeVector3(tool_position8[motion_cnt2][0], 
                                                  tool_position8[motion_cnt2][1], 
                                                  tool_position8[motion_cnt2][2]);
      target_pose.orientation = OM_MATH::makeMatrix3(1.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,
                                                    0.0f, 0.0f, 1.0f);

      chain.setPose(TOOL, target_pose, 0.5);

      if (tool_position8[motion_cnt2][3] == 1){
        // digitalWrite(RELAY_PIN, HIGH);      //suction on
        // suction_on = true;
      } 
      else{
        // digitalWrite(RELAY_PIN, LOW);      //suction off
        // suction_on = false;
      }

      motion_cnt2++;
      
      if (motion_cnt2 == 82){
        // drawing = true;  // why not wokring...?
        drawing_flag = 1;
      }
    }

    else 
    {
      Vector3f temp_current_position;
      temp_current_position(0) = 0.055f;
      temp_current_position(1) = -0.035f;
      temp_current_position(2) = 0.025f;

      if (motion_page == CIRCLEEDGE2)
      {
        chain.drawInit(CIRCLEEDGE2, move_time, p_init_arg);
        chain.setRadiusForDrawing(CIRCLEEDGE2, radius);
        chain.setStartPositionForDrawing(CIRCLEEDGE2, temp_current_position);
        chain.setStartAngularPositionForDrawing(CIRCLEEDGE2, start_angular_position);
        chain.draw(CIRCLEEDGE2);
        motion_page = 22;
      }
      else
      {
        chain.drawInit(CIRCLEEDGE, move_time, p_init_arg);
        chain.setRadiusForDrawing(CIRCLEEDGE, radius);
        chain.setStartPositionForDrawing(CIRCLEEDGE, temp_current_position);
        chain.setStartAngularPositionForDrawing(CIRCLEEDGE, start_angular_position);
        chain.draw(CIRCLEEDGE);
        motion_page++;
        // drawing = false;  // why not working??
        drawing_flag = 0;
      }
    }
  }
}




void test3()
{
  if (chain.moving() || chain.drawing())
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
      chain.drawInit(CIRCLEEDGE, move_time, p_init_arg);
      chain.setRadiusForDrawing(CIRCLEEDGE, radius);
      chain.setStartPositionForDrawing(CIRCLEEDGE, temp_current_position);
      chain.setStartAngularPositionForDrawing(CIRCLEEDGE, start_angular_position);
      chain.draw(CIRCLEEDGE);
      motion_page++;
    }
    else
    {
      chain.drawInit(CIRCLEEDGE2, move_time, p_init_arg);
      chain.setRadiusForDrawing(CIRCLEEDGE2, radius);
      chain.setStartPositionForDrawing(CIRCLEEDGE2, temp_current_position);
      chain.setStartAngularPositionForDrawing(CIRCLEEDGE2, start_angular_position);
      chain.draw(CIRCLEEDGE2);
      motion_page = 22;
    }
  }
}



#endif // DEMO_H_
