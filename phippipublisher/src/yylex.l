%{
#include <stdio.h>
#include "parser.hh"


extern void yyparse(void);
extern int yylex_destroy(void);

void 
parse_equation(char *str_input) {

    yy_scan_string(str_input);
    yyparse();
    /* to avoid leakage */
    yylex_destroy();
}



%}
%option nounput
%%
[0-9]+                  yylval=atoi(yytext); return NUMBER;
heat                    return TOKHEAT;
on|off                  yylval=!strcmp(yytext,"on"); return STATE;
target                  return TOKTARGET;
temperature             return TOKTEMPERATURE;
\n                      /* ignore end of line */;
[ \t]+                  /* ignore whitespace */;
 
%%
