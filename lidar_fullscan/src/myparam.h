#ifndef MY_PARAM_H
#define MY_PARAM_H 1


static const float ROTATION_SOLUTION_ = 0.4;
static const int POINT_PER_CIRCLE_ = (int) (360/ROTATION_SOLUTION_)+200;//900
static const int DATA_NUMBER_PER_SCAN = POINT_PER_CIRCLE_ * 16 * 2; //28800
const float VERT_ANGLE[] = {-0.2617993877991494,
                                                0.017453292519943295,
                                                -0.22689280275926285,
                                                0.05235987755982989,
                                                -0.19198621771937624,
                                                0.08726646259971647,
                                                -0.15707963267948966,
                                                0.12217304763960307,
                                                -0.12217304763960307,
                                                0.15707963267948966,
                                                -0.08726646259971647,
                                                0.19198621771937624,
                                                -0.05235987755982989,
                                                0.22689280275926285,
                                                -0.017453292519943295,
                                                0.2617993877991494};
const float VERT_ANGLE2[] = {
    -0.2617993877991494,
    -0.22689280275926285,
    -0.19198621771937624,
    -0.15707963267948966,
    -0.12217304763960307,
    -0.08726646259971647,
    -0.05235987755982989,
    -0.017453292519943295,
    0.017453292519943295,
    0.05235987755982989,
    0.08726646259971647,
    0.12217304763960307,
    0.15707963267948966,
    0.19198621771937624,
    0.22689280275926285,
    0.2617993877991494
};

static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f; /**< degrees */
static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

/** According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
 *  valid packets with readings up to 130.0. */
static const float DISTANCE_MAX = 130.0f;        /**< meters */
static const float DISTANCE_RESOLUTION = 0.002f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX
                                         / DISTANCE_RESOLUTION + 1.0);
/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;


/** Special Defines for VLP16 support **/
static const int    VLP16_FIRINGS_PER_BLOCK =   2;
static const int    VLP16_SCANS_PER_FIRING  =  16;
static const float  VLP16_BLOCK_TDURATION   = 110.592f;   // [µs]
static const float  VLP16_DSR_TOFFSET       =   2.304f;   // [µs]
static const float  VLP16_FIRING_TOFFSET    =  55.296f;   // [µs]


/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block
{
  uint16_t header;        ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
  uint8_t  data[BLOCK_DATA_SIZE];
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t  bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;


#endif
