#pragma once

#include "GBox.h"

#include <array>

namespace gbox_test
{
inline BoxBoardPieces emptyBoardPieces()
{
    BoxBoardPieces board = {};
    board.fill(BoxPieceId::EMPTY);
    return board;
}

inline constexpr std::array<BoxPieceId, BOX_TOTAL_SLOTS> kDemoBoardPieces = {
    BoxPieceId::FORWARD,    BoxPieceId::TURN_RIGHT, BoxPieceId::LOOP_CALL, BoxPieceId::FORWARD,
    BoxPieceId::STOP,       BoxPieceId::EMPTY,      BoxPieceId::EMPTY,     BoxPieceId::EMPTY,
    BoxPieceId::EMPTY,      BoxPieceId::EMPTY,      BoxPieceId::EMPTY,     BoxPieceId::EMPTY,
    BoxPieceId::EMPTY,      BoxPieceId::EMPTY,      BoxPieceId::TURN_LEFT, BoxPieceId::FORWARD,
    BoxPieceId::TURN_RIGHT, BoxPieceId::STOP,       BoxPieceId::EMPTY,     BoxPieceId::EMPTY,
};

inline constexpr std::array<BoxPieceId, BOX_TOTAL_SLOTS> kOverflowBoardPieces = {
    BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,  BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,
    BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,  BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,
    BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,  BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,
    BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,  BoxPieceId::FORWARD,   BoxPieceId::BACKWARD,
    BoxPieceId::TURN_LEFT, BoxPieceId::TURN_RIGHT, BoxPieceId::STOP,      BoxPieceId::FORWARD,
};

inline void injectBoardPieces(GBox &box, const BoxBoardPieces &boardPieces)
{
    box.injectBoardPieces(boardPieces);
}

inline void injectDemoBoard(GBox &box)
{
    injectBoardPieces(box, kDemoBoardPieces);
}

inline void injectEmptyBoard(GBox &box)
{
    injectBoardPieces(box, emptyBoardPieces());
}

inline void injectOverflowBoard(GBox &box)
{
    injectBoardPieces(box, kOverflowBoardPieces);
}
}
