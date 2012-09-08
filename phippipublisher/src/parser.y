%{
#include <stdio.h>
#include <string.h>
void 
yyerror(const char *str) {
    fprintf(stderr,"yyerror: '%s'\n",str);
}

int 
yywrap() {
    return 1;
}

extern int yylex();
extern int yyparse();
extern FILE *yyin;

%}
%token NUMBER HEAT ONOFF TARGET TEMPERATURE SETCURSOR
%token ACC GYRO LMOTOR RMOTOR BATTERY CAPACITOR BALL CHARGER
%token FLOAT MELEXISR MELEXISL PANDA ODOMETRY FIVEVLDO
%%
commands: /* empty */
         | commands command;

command: charger_switch |
         target_set|
	 gyro|
	 odometry|
	charger|panda
;

charger_switch: CHARGER ONOFF {
             printf("\tCharger on or off\n");
         };

target_set: TARGET TEMPERATURE NUMBER {
             printf("\tTemperature set\n");
         };
gyro: SETCURSOR GYRO ' x:' FLOAT ' y:'  FLOAT ' z:' FLOAT {
             printf("\tGyro\n");

};
odometry: SETCURSOR ODOMETRY ' dx:' FLOAT ' dy:' FLOAT ' yaw:' FLOAT {
             printf("\tOdometry\n");


};
panda: SETCURSOR PANDA ONOFF {
             printf("\tPanda\n");
};
charger: SETCURSOR CHARGER ONOFF {
             printf("\tCharger\n");
};
%%
