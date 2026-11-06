#pragma once

#include "CommandPacket.h"

/* Program building from a scanned board, shared between the production GBox
 * and the bench so both produce the same payload from the same board: the
 * main zone (BOX_MAIN_SLOTS) is walked in order, and a LOOP_CALL in that
 * zone replays the loop zone (BOX_LOOP_SLOTS) in place. An INVALID slot
 * aborts the build, an EMPTY board yields an empty packet. */

bool buildProgramFromPieces(const BoxBoardPieces &boardPieces, CommandPacket &packet,
                            bool &isEmpty);
