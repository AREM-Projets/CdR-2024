#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include <stdint.h>
#include <stdio.h>

#define UNUSED_ARG 0

/* Max length of a command name */
#define COMMAND_NAME_MAX_LENGTH 2

/* DO NOT MODIFY THIS ALONE. All this library is build on the fact that commands take up to 3 args that are all ints. 
 * If you still want to modify it, see guide above. TODO: write the guide
 */ 
#define COMMAND_MAX_ARGS 3

/* Maximum number of valid commands */
#define MAX_COMMAND_COUNT 20

enum CommandParserError_t
{
    PARSER_OK,
    COMMAND_DOES_NOT_EXIST,
    TOO_MANY_ARGS,
    COMMAND_NAME_TOO_LONG

};

struct Command_t
{
    int32_t (*command_function)(int32_t arg1, int32_t arg2, int32_t arg3);
    char name[COMMAND_NAME_MAX_LENGTH + 1];
    uint8_t nb_args;
};

class commandParser
{
private:
    char _command_name[COMMAND_NAME_MAX_LENGTH + 1] = {}; /**< +1 to the size for the '\0' */ 
    int32_t _command_args[COMMAND_MAX_ARGS] = {0}; /**< All args should be ints because I decided it */
    Command_t _valid_commands[MAX_COMMAND_COUNT] = {}; /**< All valid commands */
    uint32_t _valid_command_count;

    CommandParserError_t parseString(const char *string);
    CommandParserError_t execute();
    
public:
    commandParser();
    CommandParserError_t addCommand(int32_t (*command_function)(int32_t arg1, int32_t arg2, int32_t arg3), const char *name, uint8_t nb_args = 0);
    CommandParserError_t removeCommand(const char *name);
    CommandParserError_t parseAndExecute(const char *string);
};


int32_t atoi(const char *str, uint32_t length);

#endif /* COMMANDPARSER_H */
