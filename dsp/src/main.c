/* xdctools header files */
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* package header files */
#include <ti/ipc/Ipc.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <trik/sensors/dsp_server.h>

/* private functions */
static Void smain(UArg arg0, UArg arg1);

Int main(Int argc, Char* argv[]) {
  Error_Block eb;
  Task_Params taskParams;

  Log_print0(Diags_ENTRY, "--> main:");

  /* must initialize the error block before using it */
  Error_init(&eb);

  /* create main thread (interrupts not enabled in main on BIOS) */
  Task_Params_init(&taskParams);
  taskParams.instance->name = "smain";
  taskParams.arg0 = (UArg) argc;
  taskParams.arg1 = (UArg) argv;
  taskParams.stackSize = 0x1000;
  Task_create(smain, &taskParams, &eb);

  if (Error_check(&eb))
    System_abort("main: failed to create application startup thread");

  /* start scheduler, this never returns */
  BIOS_start();

  /* should never get here */
  Log_print0(Diags_EXIT, "<-- main:");
  return (0);
}

/*
 *  ======== smain ========
 */
Void smain(UArg arg0, UArg arg1) {
  Int status = 0;
  Error_Block eb;

  Log_print0(Diags_ENTRY | Diags_INFO, "--> smain:");
  Error_init(&eb);
  Diags_setMask("Server+F");

  status = trik_init_dsp_server();
  if (status < 0)
    goto leave;

  status = trik_start_dsp_server();
  if (status < 0)
    goto leave;

  status = trik_destroy_dsp_server();
  if (status < 0)
    goto leave;

leave:
  Log_print1(Diags_EXIT, "<-- smain: %d", (IArg) status);
  return;
}
