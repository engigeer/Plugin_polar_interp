#include "grbl/hal.h"
#include "polar_interp.h"

static on_report_options_ptr on_report_options;

static void report_options(bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Polar Interpolation", "0.01");
}

void polar_interp_init(void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}