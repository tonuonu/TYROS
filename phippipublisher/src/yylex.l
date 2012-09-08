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


"Acceleration:"		printf("ACC \n");return ACC;
"Gyroscope:"            printf("GYRO \n");return GYRO;
"Left motor:"           printf("LMOTOR \n");return LMOTOR;
"Right motor:"          printf("RMOTOR \n");return RMOTOR;
"Battery:"              printf("BATTERY \n");return BATTERY;
"Capacitor:"            printf("CAPACITOR \n");return CAPACITOR;
"Charger:"              printf("CHARDER\n");return CHARGER;
"Ball:"                 printf("BALL \n");return BALL;
"Melexis R:"            printf("MELEXIS R \n");return MELEXISR;
"Melexis L:"            printf("MELEXIS L \n");return MELEXISL;
"Panda:"                printf("PANDA \n");return PANDA;
"Odometry:"             printf("ODOMETRY \n");return ODOMETRY;
"5V LDO:"               printf("5V LDO\n");return FIVEVLDO;
\[[0-9]+;[0-9]+H	printf("SETCURSOR \n");return SETCURSOR;
[[0-9]+\.[0-9]+		printf("FLOAT\n");return FLOAT;
[0-9]+                  printf("NUMBER '%s'\n",yytext);yylval=atoi(yytext); return NUMBER;
on|off                  printf("ONOFF\n");yylval=!strcmp(yytext,"on"); return ONOFF;
\n                      printf("NEWLINE\n");/* ignore end of line */;
[ \t]+                  printf("WHITE\n");/* ignore whitespace */;
[a-zA-Z0-9]+	        printf("WTF '%s'\n",yytext);/* ignore the rest printable */;
.	                printf("WTFchar '%c'\n",yytext[0]);/* ignore the rest nonprintable */;
 
%%
