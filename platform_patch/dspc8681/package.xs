/*
 *  ======== package.xs ========
 *
 */


/*
 *  ======== Package.getLibs ========
 *  This function is called when a program's configuration files are
 *  being generated and it returns the name of a library appropriate
 *  for the program's configuration.
 */

function getLibs(prog)
{
    var lib = "";

    if (prog.build.target.suffix == "e66")
    {
        if (this.profile.match(/debug/))
        {
            lib = "platform_lib/lib/debug/ti.platform.dspc8681.ae66";
        } else
        {
            lib = "platform_lib/lib/release/ti.platform.dspc8681.ae66";
        }
    }
    else if (prog.build.target.suffix == "e66e")
    {
        if (this.profile.match(/debug/))
        {
            lib = "platform_lib/lib/debug/ti.platform.dspc8681.ae66e";
        } else
        {
            lib = "platform_lib/lib/release/ti.platform.dspc8681.ae66e";
        }
    }

    if (java.io.File(this.packageBase + lib).exists()) {
        return lib;
    }

    /* could not find any library, throw exception */
    throw Error("Library not found: " + this.packageBase + lib);
}

/*
 *  ======== package.close ========
 */
function close()
{    
    if (xdc.om.$name != 'cfg') {
        return;
    }
}

