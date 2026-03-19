#pragma once

enum CommandType
{
    CMD_NONE = 0,
    CMD_HOME,
    CMD_SET_POS,
    CMD_SET_ANG
};

CommandType receiveCommand();