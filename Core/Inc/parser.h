/*
 * parser.h
 *
 *  Created on: 14 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#include "stdint.h"

#define MAX_TOKENS 11

typedef struct {
	char *token_name;
	uint64_t token_value;
} token;

typedef struct {
	char* filedata;
	token tokens[MAX_TOKENS];
	uint8_t tokens_count;
	uint16_t filelength;
} configfile;

void open_config(configfile *config, char* filedata);
void tokenize_config(configfile *config);
void write_config(configfile *config);
void close_config(configfile *config);

void add_token(configfile *config, char *token_name, uint64_t token_value);
void edit_token(configfile *config, char *token_name, uint64_t token_value);
token get_token_by_name(configfile *config, char *token_name);
token get_token_by_index(configfile *config, uint8_t index);

#endif /* INC_PARSER_H_ */
