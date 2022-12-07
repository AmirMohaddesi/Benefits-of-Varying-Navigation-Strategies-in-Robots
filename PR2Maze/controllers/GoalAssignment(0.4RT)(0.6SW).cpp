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
#include <sstream>

using namespace std;

int main()
{
	string command = "del /Q ";
	string path = "\AssignedGoals*";
	system(command.append(path).c_str());
	command = "del /Q ";
	path = "\AssGoals*";
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
    
    int rr = 0;
    int yy = 0;
    for(int i=0;i<n;i++){
        ofstream of("AssignedGoals"+std::to_string(rr+1)+".txt",ios::app);
        ofstream off("AssGoals"+std::to_string(yy+1)+".txt",ios::app);
		if((rand()%10+1)<=4){
			if(i<m){
//				for(int j=0 ; j!=a[i] ; j=(j+1)%n){
//					of<<j<<" ";
//				}	
			}else{
				for(int l=(a[i-m]+1)%n;l!=a[i];l=(l+1)%n){
					of<<l<<" ";
				}
			}
		}
		
        of<<a[i]<<" ";
        off<<a[i]<<" ";
        rr=((rr+1)%m);
        yy=((yy+1)%m);
    }

    return 0;
}
