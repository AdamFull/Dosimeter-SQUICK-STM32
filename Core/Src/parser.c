/*
 * parser.c
 *
 *  Created on: 14 авг. 2020 г.
 *      Author: VeAnIlAn
 */

#include "parser.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "ctype.h"
#include "stdio.h"

void append(char* s, char c)
{

    int len = strlen(s);
    s[len] = c;
    s[len+1] = '\0';
}

void open_config(configfile *config, char* filedata){
	memset(config->filedata, 0, MAX_FILE_SIZE * sizeof(char));
	strcpy(config->filedata, filedata);
	config->tokens_count = 0;
	config->filelength = strlen(config->filedata) * sizeof(char);
}

void tokenize_config(configfile *config){
	uint16_t cursor = 0;
	char *filedata = config->filedata;
	uint16_t len = config->filelength;
	char parsed_name[32], parsed_num[32];
	memset(parsed_name, 0, 32 * sizeof(char));
	memset(parsed_num, 0, 32 * sizeof(char));
	bool is_number_parsed = false;

	if(filedata != NULL)
	while(true){
		if(cursor > len) break;
		char current = filedata[cursor];

		if(is_number_parsed){
			char * pEnd;
			add_token(config, parsed_name, strtoull(parsed_num, &pEnd, 10));
			memset(parsed_name, 0, strlen(parsed_name) * sizeof(char));
			memset(parsed_num, 0, strlen(parsed_num) * sizeof(char));
			is_number_parsed = false;
		}

		if(isalpha(current)){
			while(isalpha(current) || current == '_'){
				append(parsed_name, current);
				cursor++;
				current = filedata[cursor];
			}
		}else if(isdigit(current)){
			while(isdigit(current)){
				append(parsed_num, current);
				cursor++;
				current = filedata[cursor];
			}
			is_number_parsed = true;
		}
		cursor++;
	}
}

void write_config(configfile *config){
	//calculating new space
	memset(config->filedata, 0, MAX_FILE_SIZE * sizeof(char));

	char buffer[64];

	for(unsigned i = 0; i < MAX_TOKENS; i++){
		memset(buffer, 0, 64);
		sprintf(buffer, "%s=%llu\n", config->tokens[i].token_name, (uint64_t)config->tokens[i].token_value);
		strcat(config->filedata, buffer);
	}

}

void close_config(configfile *config){

}

void add_token(configfile *config, char *token_name, uint64_t token_value){
	if(config->tokens_count < MAX_TOKENS){
		token new_token;
		memset(new_token.token_name, 0, MAX_TOKEN_SIZE);
		strcpy(new_token.token_name, token_name);

		new_token.token_value = token_value;
		config->tokens[config->tokens_count] = new_token;
		config->tokens_count++;
	}
}

void edit_token(configfile *config, char *token_name, uint64_t token_value){
	uint8_t token_counter = 0;
	while(true){
		if(token_counter > MAX_TOKENS) break;
		if(strcmp(config->tokens[token_counter].token_name, token_name) == 0){
			config->tokens[token_counter].token_value = token_value;
			break;
		}else{
			token_counter++;
			continue;
		}
	}
}

token get_token_by_name(configfile *config, char *token_name){
	uint8_t token_counter = 0;
	while(true){
		if(token_counter > MAX_TOKENS) break;
		if(strcmp(config->tokens[token_counter].token_name, token_name) == 0){
			return config->tokens[token_counter];
		}else{
			token_counter++;
			continue;
		}
	}
}

token get_token_by_index(configfile *config, uint8_t index){
	return config->tokens[index];
}
