#pragma once

#include "GBox.h"

namespace gbox_test
{

inline constexpr BoxPieceId kEmptyBoardPieces[] = {
    BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY,
    BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY,
    BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY,
    BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY, BoxPieceId::EMPTY,
};

inline constexpr BoxPieceId kDemoBoardPieces[] = {
    BoxPieceId::FORWARD,    BoxPieceId::TURN_RIGHT, BoxPieceId::LOOP_CALL, BoxPieceId::FORWARD,
    BoxPieceId::STOP,       BoxPieceId::EMPTY,      BoxPieceId::EMPTY,     BoxPieceId::EMPTY,
    BoxPieceId::EMPTY,      BoxPieceId::EMPTY,      BoxPieceId::EMPTY,     BoxPieceId::EMPTY,
    BoxPieceId::EMPTY,      BoxPieceId::EMPTY,      BoxPieceId::TURN_LEFT, BoxPieceId::FORWARD,
    BoxPieceId::TURN_RIGHT, BoxPieceId::STOP,       BoxPieceId::EMPTY,     BoxPieceId::EMPTY,
};

inline constexpr BoxPieceId kOverflowBoardPieces[] = {
    BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,  BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,
    BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,  BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,
    BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,  BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,
    BoxPieceId::LOOP_CALL, BoxPieceId::LOOP_CALL,  BoxPieceId::FORWARD,   BoxPieceId::BACKWARD,
    BoxPieceId::TURN_LEFT, BoxPieceId::TURN_RIGHT, BoxPieceId::STOP,      BoxPieceId::FORWARD,
};

static_assert(sizeof(kEmptyBoardPieces) == sizeof(BoxBoardPieces),
              "Board piece arrays must match BoxBoardPieces size");

inline void fillEmptyBoard(BoxPieceId (&board)[BOX_TOTAL_SLOTS])
{
    for (std::size_t i = 0; i < BOX_TOTAL_SLOTS; i++) {
        board[i] = BoxPieceId::EMPTY;
    }
}

inline void injectBoardPieces(GBox &box, const BoxPieceId *board)
{
    box.injectBoardPieces(reinterpret_cast<const BoxBoardPieces &>(*board));
}

inline void injectDemoBoard(GBox &box)
{
    injectBoardPieces(box, kDemoBoardPieces);
}

inline void injectEmptyBoard(GBox &box)
{
    injectBoardPieces(box, kEmptyBoardPieces);
}

inline void injectOverflowBoard(GBox &box)
{
    injectBoardPieces(box, kOverflowBoardPieces);
}
}
