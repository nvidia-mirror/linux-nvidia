/*
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef AON_IVC_DBG_MESSAGES_H
#define AON_IVC_DBG_MESSAGES_H

#define AON_BOOT		    0
#define AON_PING		    1
#define AON_QUERY_TAG		    2
#define AON_MODS_LOOPS_TEST	    3
#define AON_MODS_RESULT		    4
#define AON_MODS_CRC		    5
#define AON_REQUEST_TYPE_MAX	    5

#define AON_DBG_STATUS_OK	    0
#define AON_DBG_STATUS_ERROR	    1

/**
 * @brief ping message request
 *
 * Structure that describes the ping message request.
 *
 * @challenge	    Arbitrarily chosen value. Response to ping is
 *		    computed based on this value.
 */
struct aon_ping_req {
	u32 challenge;
};

/**
 * @brief response to the ping request
 *
 * Structure that describes the response to ping request.
 *
 * @reply	    Response to ping request with challenge left-shifted
 *		    by 1 with carry-bit dropped.
 */
struct aon_ping_resp {
	u32 reply;
};

/**
 * @brief response to the query tag request
 *
 * This struct is used to extract the tag/firmware version of the AON.
 *
 * @tag		    array to store tag information.
 */
struct aon_query_tag_resp {
	u8 tag[32];
};

/**
 * @brief mods test request
 *
 * This struct is used to send the loop count to perform the mods test
 * on the target.
 *
 * @loops	    number of times mods test should be run
 */
struct aon_mods_req {
	u32 loops;
};

/**
 * @brief mods test crc response
 *
 * This struct is used to send the CRC32 of the AON text section to the target.
 * Fields:
 *
 * @crc		    CRC32 of the text section.
 */
struct aon_mods_crc_resp {
	u32 crc;
};

/**
 * @brief aon dbg request
 *
 * This struct encapsulates the type of the request and the reaonctive
 * data associated with that request.
 *
 * @req_type	    indicates the type of the request
 * @data	    data needed to send for the request
 */
struct aon_dbg_request {
	u32 req_type;
	union {
		struct aon_ping_req ping_req;
		struct aon_mods_req mods_req;
	} data;
};

/**
 * @brief aon dbg response
 *
 * This struct encapsulates the type of the response and the reaonctive
 * data associated with that response.
 *
 * @resp_type	    indicates the type of the response.
 * @status	    response in regard to the request i.e success/failure.
 *		    In case of mods, this field is the result.
 * @data	    data associated with the response to a request.
 */
struct aon_dbg_response {
	u32 resp_type;
	u32 status;
	union {
		struct aon_ping_resp	  ping_resp;
		struct aon_query_tag_resp tag_resp;
		struct aon_mods_crc_resp  crc_resp;
	} data;
};

#endif
