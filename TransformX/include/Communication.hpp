#pragma once

/**
 * @brief Types of commands received via serial
 */
enum CommandType
{
    CMD_NONE,
    CMD_HOME,
    CMD_SET_POS,
    CMD_SET_ANG,
    CMD_WALK
};

/**
 * @brief Command structure
 */
struct Command
{
    CommandType type;

    float x, y, z;
    int leg;
    float t1, t2, t3;
};

/**
 * @brief Parse incoming serial command
 *
 * @return Parsed command
 */
Command receiveCommand();