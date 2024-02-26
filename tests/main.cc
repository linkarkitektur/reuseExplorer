#include <nexus/Nexus.hh>


int main(int argc, char** argv)
{
    nx::Nexus nx;
    nx.applyCmdArgs(argc, argv);
    return nx.run();
}
