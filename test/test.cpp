#include <iostream>
#include <string> 
using namespace std;

int main()
{
    string s = "c1,2.2,3.2";

   if(s[0] == 'c')
    {
        int delim = s.find(',');
        int last = s.find(',',delim+1);
    
        string a = s.substr(1,delim-1); 
        string b = s.substr(delim+1,last-1);
        string c = s.substr(last+1);

        cout << a << " | " << b << " | " << c << endl; 

    }


    return 0;
}