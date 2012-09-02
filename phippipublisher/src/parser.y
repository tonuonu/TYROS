%{
#include <stdio.h>
#include <string.h>
void 
yyerror(const char *str) {
    fprintf(stderr,"error: %s\n",str);
}

int 
yywrap() {
    return 1;
}

extern int yylex();
extern int yyparse();
extern FILE *yyin;

%}
%token NUMBER TOKHEAT STATE TOKTARGET TOKTEMPERATURE
%%
commands: /* empty */
         | commands command;

command: heat_switch |
         target_set;

heat_switch: TOKHEAT STATE {
             printf("\tHeat turned on or off\n");
         };

target_set: TOKTARGET TOKTEMPERATURE NUMBER {
             printf("\tTemperature set\n");
         };
%%
