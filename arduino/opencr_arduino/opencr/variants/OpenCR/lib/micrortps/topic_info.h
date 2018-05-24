/*
 * micrortps.h
 *
 *  Created on: May 21, 2018
 *      Author: Kei
 */

#ifndef TOPIC_INFO_H_
#define TOPIC_INFO_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "micrortps.h"

typedef struct TopicInfo{
    char* profile;
    SerializeTopic serialize_func;
    DeserializeTopic deserialize_func;
} TopicInfo_t;



#ifdef __cplusplus
 }
#endif

#endif /* TOPIC_INFO_H_ */
