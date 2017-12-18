#include <map>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

void parse (string);

map<string, string> options;

void parse(string strfile) {
    filebuf fb;
    if (fb.open ("conf.ini", ios::in)) {
        istream cfgfile(&fb);

        for (string line; getline(cfgfile, line); ) {
             istringstream iss(line);
             string id, eq, val;

            bool error = false;

            if (!(iss >> id)) {
                error = true;
            }
            else if (id[0] == '#') {
                continue;
            }
            else if (!(iss >> eq >> val >>  ws) || eq != "=" || iss.get() != EOF) {
                error = true;
            }

            if (error) {
                cout << "Error" << endl;
                // do something appropriate: throw, skip, warn, etc.
            }
            else {
                options[id] = val;
            }
        }
        fb.close();
    }
}

int main () {
    parse("conf.ini");

    for (map<string,string>::iterator it = options.begin(); it != options.end(); it++) {
        cout << "clave: " << it->first << " - valor: " << it->second << endl;
    }

    cout << "a verrr" << options["LINE1_POSITION_PERCENTAGE"];

    return 0;
}
