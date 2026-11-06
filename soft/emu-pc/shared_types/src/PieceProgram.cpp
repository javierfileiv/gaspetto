#include "PieceProgram.h"

#include <stddef.h>
#include <stdint.h>

namespace
{

CommandId pieceToCommand(BoxPieceId piece)
{
    switch (piece) {
    case BoxPieceId::FORWARD:
        return CommandId::MOTOR_FORWARD;
    case BoxPieceId::BACKWARD:
        return CommandId::MOTOR_BACKWARD;
    case BoxPieceId::TURN_RIGHT:
        return CommandId::MOTOR_TURN_RIGHT;
    case BoxPieceId::TURN_LEFT:
        return CommandId::MOTOR_TURN_LEFT;
    default:
        return CommandId::NONE;
    }
}

bool appendCommand(CommandPacket &packet, CommandId command)
{
    if (packet.count >= BOX_MAX_PAYLOAD_COMMANDS) {
        return false;
    }

    packet.commands[packet.count] = static_cast<uint8_t>(command);
    ++packet.count;
    return true;
}

/* An EMPTY piece contributes nothing but keeps the board buildable, any
 * other piece without a command (LOOP_CALL outside a main slot) does not */
bool appendPieceCommand(CommandPacket &packet, BoxPieceId piece)
{
    const CommandId command = pieceToCommand(piece);

    if (command == CommandId::NONE) {
        return piece == BoxPieceId::EMPTY;
    }

    return appendCommand(packet, command);
}

} /* namespace */

bool buildProgramFromPieces(const BoxBoardPieces &boardPieces, CommandPacket &packet, bool &isEmpty)
{
    BoxPieceId loopPieces[BOX_LOOP_SLOTS] = {};

    packet.count = 0;
    for (size_t index = 0; index < BOX_MAX_PAYLOAD_COMMANDS; ++index) {
        packet.commands[index] = static_cast<uint8_t>(CommandId::NONE);
    }

    for (size_t slot = 0; slot < BOX_LOOP_SLOTS; ++slot) {
        loopPieces[slot] = boardPieces[BOX_MAIN_SLOTS + slot];
        if (loopPieces[slot] == BoxPieceId::INVALID) {
            isEmpty = false;
            return false;
        }
        if (loopPieces[slot] == BoxPieceId::LOOP_CALL) {
            loopPieces[slot] = BoxPieceId::EMPTY;
        }
    }

    for (size_t slot = 0; slot < BOX_MAIN_SLOTS; ++slot) {
        const BoxPieceId piece = boardPieces[slot];

        if (piece == BoxPieceId::EMPTY) {
            continue;
        }

        if (piece == BoxPieceId::INVALID) {
            isEmpty = false;
            return false;
        }

        if (piece == BoxPieceId::LOOP_CALL) {
            for (size_t loopSlot = 0; loopSlot < BOX_LOOP_SLOTS; ++loopSlot) {
                if (loopPieces[loopSlot] == BoxPieceId::EMPTY) {
                    continue;
                }

                if (!appendPieceCommand(packet, loopPieces[loopSlot])) {
                    isEmpty = false;
                    return false;
                }
            }
            continue;
        }

        if (!appendPieceCommand(packet, piece)) {
            isEmpty = false;
            return false;
        }
    }

    isEmpty = packet.count == 0;
    return !isEmpty;
}
