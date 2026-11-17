#pragma once

#include "CommandPacket.h"

/* Program building from a scanned board, shared between the production GBox
 * and the bench so both produce the same payload from the same board: the
 * main zone (BOX_MAIN_SLOTS) is walked in order, and a LOOP_CALL in that
 * zone replays the loop zone (BOX_LOOP_SLOTS) in place. The walk stops at
 * the first EMPTY wherever it lands, in both zones: the pieces form a
 * contiguous run, so an empty slot ends the sequence and whatever follows
 * it is out of the program - what was already walked stays. An INVALID
 * slot aborts the build, an EMPTY board yields an empty packet. */

bool buildProgramFromPieces(const BoxBoardPieces &boardPieces, CommandPacket &packet,
                            bool &isEmpty);
