#include <nexus/app.hh>

APP("myapp")
{
    // simple built-in argparse library
    int n = 10;
    float t = 1.0f;
    auto args = nx::args("My App", "just a simple app")
        .version("0.0.1-alpha")
        .add("b", "some bool")
        .add({"f", "flag"}, "some flag")
        .group("vars")
        .add(n, "n", "iterations")
        .add(t, {"t", "time"}, "time parameter");

    if (!args.parse()) // takes arguments from app
        return;

    auto b = args.has("b");

    auto posargs = args.positional_args();

}