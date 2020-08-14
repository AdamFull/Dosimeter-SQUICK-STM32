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

void append(char* s, char c)
{

    int len = strlen(s);
    s[len] = c;
    s[len+1] = '\0';
}

void open_config(configfile *config, char* filedata){
	config->filelength = strlen(filedata);
	config->filedata = malloc(config->filelength);
	memset(config->filedata, 0, config->filelength);
	strcpy(config->filedata, filedata);
	config->tokens_count = 0;
}

void tokenize_config(configfile *config){
	uint16_t cursor = 0;
	char *filedata = config->filedata;
	uint16_t len = config->filelength;
	char *parsed_name = malloc(32 * sizeof(char)), *parsed_num = malloc(32 * sizeof(char));
	memset(parsed_name, 0, strlen(parsed_name));
	memset(parsed_num, 0, strlen(parsed_num));
	bool is_number_parsed = false;

	if(filedata != NULL)
	while(true){
		if(cursor > len) break;
		char current = filedata[cursor];

		if(is_number_parsed){
			char * pEnd;
			add_token(config, parsed_name, strtoull(parsed_num, &pEnd, 10));
			memset(parsed_name, 0, strlen(parsed_name));
			memset(parsed_num, 0, strlen(parsed_num));
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
	free(parsed_name);
	free(parsed_num);
	free(filedata);
}

void write_config(configfile *config){
	free(config->filedata);
	//calculating new space
	uint16_t new_size = 0;
	for(unsigned i = 0; i < MAX_TOKENS; i++){
		char buff[10];
		new_size += 1 + strlen(config->tokens[i].token_name) * sizeof(char);
		memcpy(buff, (char*)&config->tokens[i].token_value, 10);
		new_size += 1 + strlen(buff) * sizeof(char);
	}

	config->filedata = malloc(new_size);
	memset(config->filedata, 0, new_size);
	config->filelength = new_size;

	for(unsigned i = 0; i < MAX_TOKENS; i++){
		char buff[10];
		strcat(config->filedata, config->tokens[i].token_name);
		append(config->filedata, '=');
		memcpy(buff, (char*)&config->tokens[i].token_value, 10);
		strcat(config->filedata, buff);
		append(config->filedata, '\n');
	}

}

void close_config(configfile *config){
	free(config->filedata);
	for(unsigned i = 0; i < MAX_TOKENS; i++){
		free(config->tokens[i].token_name);
	}
}

void add_token(configfile *config, char *token_name, uint64_t token_value){
	if(config->tokens_count < MAX_TOKENS){
		token new_token;
		uint8_t name_size = strlen(token_name);
		new_token.token_name = malloc(name_size);
		memset(new_token.token_name, 0, name_size);
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
		token current_token = config->tokens[token_counter];
		if(strcmp(current_token.token_name, token_name) == 0){
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
		token current_token = config->tokens[token_counter];
		if(strcmp(current_token.token_name, token_name) == 0){
			return current_token;
		}else{
			token_counter++;
			continue;
		}
	}
}

token get_token_by_index(configfile *config, uint8_t index){
	return config->tokens[index];
}
