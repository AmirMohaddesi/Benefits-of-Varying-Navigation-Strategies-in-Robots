/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <time.h>

using namespace std;

int main()
{
	string command = "del /Q ";
	string path = "\AssignedGoals*";
	system(command.append(path).c_str());
    int n;
    int m;
    cout<<"Enter number of goals"<<endl;
    cin>>n;
    cout<<"Enter number of robots"<<endl;
    cin>>m;
    int a[n];
    for(int i=0;i<n;i++){
        a[i]=i;
    }

    for(int i=0;i<n;i++){
        cout<<a[i]<<",";
    }
    cout<<endl;
    srand(time(0));
    random_shuffle(&a[0],&a[n]);
    
    for(int i=0;i<n;i++){
        cout<<a[i]<<",";
    }
    cout<<endl;
    
    int rr=0;
    for(int i=0;i<n;i++){
        ofstream of("AssignedGoals"+std::to_string(rr+1)+".txt",ios::app);
        of<<a[i]<<" ";
        rr=((rr+1)%m);
    }

    return 0;
}
