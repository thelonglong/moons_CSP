/* Copyright 2021 Tronlong Elec. Tech. Co. Ltd. All Rights Reserved. */

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <libgen.h>

#include "parameter_parser.h"

const char * const VERSION = "1.0";

static const char short_opts [] = ":d:v";
static const struct option long_opts [] = {
    { "direction",      required_argument,    NULL, 'd' },
    { "version",        no_argument,          NULL, 'v' },
    { "help",           no_argument,          NULL, 0   },
    { 0, 0, 0, 0 }
};

static void usage(char *prog_name)
{
    printf ("Usage: %s [options]\n\n"
            "Options:\n"
            " -d | --direction          direction of rotation (foreward: 0, reverse: 1)\n"
            " -v | --version            Version Info.\n"
            " --help                    Show this message.\n\n"
            "e.g. :\n"
            "   ./%s -d 0\n"
            "\n",
            prog_name,
            prog_name);
}

/* Parsing input parameters */
bool parse_parameter(struct _Params *params, int argc, char **argv)
{
    int c = 0;

    /* Default value */
    params->direction = -1;

    while ((c = getopt_long(argc, argv, short_opts, long_opts, NULL))!= -1) {
        switch (c) {
        case 'd':
            params->direction = strtoul(optarg, 0, 10);
            break;
        case 'v':
            printf("version : %s\n", VERSION);
            exit(0);
        case 0: //--help
            usage(basename(argv[0]));
            exit(0);
        default :
            return false;
        }
    }
    
    // Some parameters MUST specify when the mode is display or save.
    if (params->direction != 0 && params->direction != 1) {
        return false;
    }
    return true;
}