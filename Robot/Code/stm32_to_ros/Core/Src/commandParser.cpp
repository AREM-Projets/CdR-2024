#include "commandParser.h"

#include <string.h>

/**
 * @brief Transform an ASCII char array to an int.
 * If it is not an int, returns 0 as default value
 *
 * @param str Char array containing an int
 * @return int32_t
 */
int32_t atoi(const char *str, uint32_t length)
{
    int32_t res = 0;
    bool is_negative = false;

    // Handle negative values
    if (str[0] == '-')
    {
        is_negative = true;
    }

    for (int32_t i = is_negative; i < (int32_t)length; i++)
    {
        if (str[i] >= 0x30 && str[i] <= 0x39) // ASCII range for 0-9
        {
            res *= 10;            // On décale le 0
            res += str[i] - 0x30; // Nouveau chiffre des unités
        }
        else
        {
            return 0; // fail : return default value
        }
    }

    if (is_negative)
    {
        return -res;
    }
    return res;
}

commandParser::commandParser()
{
    memset(this->_valid_commands, 0, MAX_COMMAND_COUNT * sizeof(Command_t));
    _valid_command_count = 0;
    strncpy(_command_name, "", COMMAND_NAME_MAX_LENGTH);
    for (uint8_t i = 0; i < COMMAND_MAX_ARGS; i++)
    {
        _command_args[i] = 0;
    }
}

/**
 * @brief Parse a string (command) to store it into this object as command_name and command_args
 *
 * @param string The string to be stored
 * @return CommandParserError_t
 */
CommandParserError_t commandParser::parseString(const char *string)
{
    uint32_t current_index = 0;

    strncpy(_command_name, "", COMMAND_NAME_MAX_LENGTH);
    for (uint8_t i = 0; i < COMMAND_MAX_ARGS; i++)
    {
        _command_args[i] = 0;
    }

    // Copy command to the command name.
    strncpy(_command_name, &string[current_index], strcspn(&string[current_index], " \n"));
    current_index += strcspn(&string[current_index], " \n") + 1; // +1 to get rid of the space

    uint32_t argc = 0;
    while ((current_index < strlen(string)) && (strcspn(&string[current_index], " \n") < strlen(string)) && (argc < 3))
    {
        // printf("Currently remaining args : %s\n", &string[current_index]);
        _command_args[argc] = atoi(&string[current_index], strcspn(&string[current_index], " \n"));
        current_index += strcspn(&string[current_index], " \n") + 1; // +1 to get rid of the space
        argc++;
    }

    // printf("Parsed... Name: %s, Args : %d %d %d \n", _command_name, _command_args[0], _command_args[1], _command_args[2]);

    return PARSER_OK;
}

/**
 * @brief Execute the currently stored command
 *
 * @return CommandParserError_t
 */
CommandParserError_t commandParser::execute()
{
    // printf("Number of valid commands : %d\n", _valid_command_count);
    for (uint32_t i = 0; i < _valid_command_count; i++)
    {
        // printf("Command %d [%s], looking for [%s]\n", i, _valid_commands[i].name, _command_name);
        if (!strcmp(_valid_commands[i].name, _command_name))
        {
            // TODO: add validation on the number of args

            // Execute the command
            _valid_commands[i].command_function(_command_args[0], _command_args[1], _command_args[2]);
            return PARSER_OK;
        }
    }
    return COMMAND_DOES_NOT_EXIST;
}

/**
 * @brief Add a command to the list of valid commands
 *
 * @param command_function function to execute when the command is executed
 * @param name name of the command (maximum length : COMMAND_NAME_MAX_LENGTH)
 * @param nb_args number of args taken by the command
 * @return CommandParserError_t
 */
CommandParserError_t commandParser::addCommand(int32_t command_function(int32_t arg1, int32_t arg2, int32_t arg3), const char *name, uint8_t nb_args)
{
    strncpy(_valid_commands[_valid_command_count].name, name, COMMAND_NAME_MAX_LENGTH);
    _valid_commands[_valid_command_count].nb_args = nb_args;
    _valid_commands[_valid_command_count].command_function = command_function;

    _valid_command_count++;
    return PARSER_OK;
}

/**
 * @brief Remove a command from the list of valid commands
 *
 * @param name Name of the command (maximum length : COMMAND_NAME_MAX_LENGTH)
 * @return CommandParserError_t
 */
CommandParserError_t commandParser::removeCommand(const char *name)
{
    for (uint32_t i = 0; i < _valid_command_count; i++)
    {
        if (!strcmp(_valid_commands[i].name, name))
        {
            // Delete the command
            strncpy(_valid_commands[_valid_command_count].name, "", COMMAND_NAME_MAX_LENGTH);
            _valid_commands[_valid_command_count].nb_args = 0;
            _valid_commands[_valid_command_count].command_function = NULL;
            return PARSER_OK;
        }
    }
    return COMMAND_DOES_NOT_EXIST;
}

/**
 * @brief Parse then execute a string as a command
 *
 * @param string the string corresponding to the executed command
 * @return CommandParserError_t
 */
CommandParserError_t commandParser::parseAndExecute(const char *string)
{
    CommandParserError_t status;
    status = parseString(string);
    if (status)
    {
        return status;
    }

    status = execute();
    if (status)
    {
        return status;
    }

    return PARSER_OK;
}