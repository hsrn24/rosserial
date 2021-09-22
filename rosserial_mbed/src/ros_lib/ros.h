/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "ros/node_handle.h"
#include "MbedHardware.h"

namespace ros
{

    class NodeHandle : public NodeHandle_<MbedHardware,
                                          ROSSERIAL_MAX_SUBSCRIBERS,
                                          ROSSERIAL_MAX_PUBLISHERS,
                                          ROSSERIAL_INPUT_MSG_BUFFER_SIZE,
                                          ROSSERIAL_OUTPUT_MSG_BUFFER_SIZE>
    {
    public:
        virtual int spinOnce() override
        {
            static uint8_t chunk_buffer[ROSSERIAL_CHUNK_SIZE];
            static uint8_t * data_ptr;

            /* restart if timed out */
            uint32_t c_time = hardware_.time();
            if ((c_time - last_sync_receive_time) > (SYNC_SECONDS * 2200))
            {
                configured_ = time_synced_ = false;
            }

            /* reset if message has timed out */
            if (mode_ != MODE_FIRST_FF)
            {
                if (c_time > last_msg_timeout_time)
                {
                    mode_ = MODE_FIRST_FF;
                }
            }

            bool tx_stop_requested = false;
            bool saw_time_msg = false;

            /* while available buffer, read data */
            while (true)
            {
                // If a timeout has been specified, check how long spinOnce has been running.
                if (spin_timeout_ > 0)
                {
                    // If the maximum processing timeout has been exceeded, exit with error.
                    // The next spinOnce can continue where it left off, or optionally
                    // based on the application in use, the hardware buffer could be flushed
                    // and start fresh.
                    if ((hardware_.time() - c_time) > spin_timeout_)
                    {
                        // Exit the spin, processing timeout exceeded.
                        return SPIN_TIMEOUT;
                    }
                }

                int num = hardware_.read(chunk_buffer, ROSSERIAL_CHUNK_SIZE);

                if (num <= 0)
                    break;

                data_ptr = chunk_buffer;

                while (num)
                {

                    checksum_ += *data_ptr;

                    if (mode_ == MODE_MESSAGE) /* message data being recieved */
                    {
                        message_in[index_++] = *data_ptr;
                        bytes_--;
                        if (bytes_ == 0) /* is message complete? if so, checksum */
                            mode_ = MODE_MSG_CHECKSUM;
                    }
                    else if (mode_ == MODE_FIRST_FF)
                    {
                        if (*data_ptr == 0xff)
                        {
                            mode_++;
                            last_msg_timeout_time = c_time + SERIAL_MSG_TIMEOUT;
                        }
                        else if (hardware_.time() - c_time > (SYNC_SECONDS * 1000))
                        {
                            /* We have been stuck in spinOnce too long, return error */
                            configured_ = time_synced_ = false;
                            return SPIN_TIMEOUT;
                        }
                    }
                    else if (mode_ == MODE_PROTOCOL_VER)
                    {
                        if (*data_ptr == PROTOCOL_VER)
                        {
                            mode_++;
                        }
                        else
                        {
                            mode_ = MODE_FIRST_FF;
                            if (configured_ == false)
                                requestSyncTime(); /* send a msg back showing our protocol version */
                        }
                    }
                    else if (mode_ == MODE_SIZE_L) /* bottom half of message size */
                    {
                        bytes_ = *data_ptr;
                        index_ = 0;
                        mode_++;
                        checksum_ = *data_ptr; /* first byte for calculating size checksum */
                    }
                    else if (mode_ == MODE_SIZE_H) /* top half of message size */
                    {
                        bytes_ += *data_ptr << 8;
                        mode_++;
                    }
                    else if (mode_ == MODE_SIZE_CHECKSUM)
                    {
                        if ((checksum_ % 256) == 255)
                            mode_++;
                        else
                            mode_ = MODE_FIRST_FF; /* Abandon the frame if the msg len is wrong */
                    }
                    else if (mode_ == MODE_TOPIC_L) /* bottom half of topic id */
                    {
                        topic_ = *data_ptr;
                        mode_++;
                        checksum_ = *data_ptr; /* first byte included in checksum */
                    }
                    else if (mode_ == MODE_TOPIC_H) /* top half of topic id */
                    {
                        topic_ += *data_ptr << 8;
                        mode_ = MODE_MESSAGE;
                        if (bytes_ == 0)
                            mode_ = MODE_MSG_CHECKSUM;
                    }
                    else if (mode_ == MODE_MSG_CHECKSUM) /* do checksum */
                    {
                        mode_ = MODE_FIRST_FF;
                        if ((checksum_ % 256) == 255)
                        {
                            if (topic_ == TopicInfo::ID_PUBLISHER)
                            {
                                requestSyncTime();
                                negotiateTopics();
                                last_sync_time = c_time;
                                last_sync_receive_time = c_time;
                                return SPIN_ERR;
                            }
                            else if (topic_ == TopicInfo::ID_TIME)
                            {
                                saw_time_msg = time_synced_ = true;
                                syncTime(message_in);
                            }
                            else if (topic_ == TopicInfo::ID_PARAMETER_REQUEST)
                            {
                                req_param_resp.deserialize(message_in);
                                param_received = true;
                            }
                            else if (topic_ == TopicInfo::ID_TX_STOP)
                            {
                                configured_ = time_synced_ == false;
                                tx_stop_requested = true;
                            }
                            else
                            {
                                if (subscribers[topic_ - 100])
                                    subscribers[topic_ - 100]->callback(message_in);
                            }
                        }
                    }

                    data_ptr++;
                    num--;
                }
            }

            /* occasionally sync time */
            if (configured_ && ((c_time - last_sync_time) > (SYNC_SECONDS * 500)))
            {
                requestSyncTime();
                last_sync_time = c_time;
            }

            return saw_time_msg ? SPIN_TIME_RECV : (tx_stop_requested ? SPIN_TX_STOP_REQUESTED : SPIN_OK);
        }
    };
}
