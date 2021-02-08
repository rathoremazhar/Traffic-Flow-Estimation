/*
 *********************************************************************
 * Muhammad Mazhar ullah Rathore@copyright                           *
 *                                                                   *
 *********************************************************************
*/


#include <iostream>
#include <fstream>
#include <deque>
#include <string>
#include <cstddef>
#include <cstdio>
#include <iterator>
#include <algorithm>
#include <vector>
#include <set>
#include <gmp.h>
#include <sys/time.h>
#include <pthread.h>
#include <mutex>
#include<unistd.h>

#include <openssl/sha.h>
#include <cmath>


//#include "bloom_filter.hpp"
#define Total_vehicles 2000
#define k 4
#define m 8000
#define n_r 5
#define BF_versions 5

double estimate_flow(int s_r,int e_r, int *BF, int t);
void find_combinations(int *comb,int t_comb, int s_r,int e_r);
void recursion_find_combinations(int a[], int reqLen, int start, int currLen, bool check[], int len, int *comb, int *c_comb);
double print_time(struct timeval *start, struct timeval *end);
bool Find_BF_Index(int id, int v_BF_idx[][k]);
void Or_operation_ByteBF(int Bf1[], int Bf2[],int Bf3[], int s);
double estimate_no_of_element_in_1ByteBF(int Bf[]);
std::size_t Calculate_Zeros_in_ByteBf(int Bf[], int s);



using namespace std;

//static int t_Bf[8000];

int main(int argc, char **argv){
	
	int v_ids[Total_vehicles];//vehicle IDs
	bool v_status[Total_vehicles];// For status whther vehicle is still on the highway?
	int v_BF_idx[Total_vehicles][k];//vehicle IDs
	int v_speeds[Total_vehicles]; // vehicle speeds
	int v_starts[Total_vehicles]; // vehicle Start Receiver
	int v_ends[Total_vehicles];//Vehicle end receiver
	struct timeval v_stime[Total_vehicles];//Vehicle time at the last passing receiver 

	struct timeval start, end, sim_stime;

	
	gettimeofday(&start,NULL);


	for(int i=0; i<Total_vehicles; i++){
		v_ids[i]=i;
		v_status[i]=false; //vehicle is not yet on the highway
		v_speeds[i]=60+rand()%20; //60-80 KM/H

		if(i<1000) 
		{
			v_starts[i]= 0;

			if(i<200) v_ends[i]= 1;
			else if(i<300) v_ends[i]= 2;
			else if(i<600) v_ends[i]= 3;
			else  v_ends[i]= 4;

		}
		else if(i<1500) 
		{
			v_starts[i]= 1;
			
			if(i<1100) v_ends[i]= 1;
			else if(i<1150) v_ends[i]= 2;
			else if(i<1350) v_ends[i]= 3;
			else v_ends[i]= 4;

		}

		else if(i<2000) 
		{
			v_starts[i]= 2;
			
			if(i<1700) v_ends[i]= 3;
			else v_ends[i]= 4;

		}

	}

	int BF[BF_versions][n_r][m]; // initialize five versions of a Bloom filter at different times for each receiver
	int ver=0;
	for (ver=0;ver<BF_versions; ver++){
		for(int g=0; g<n_r; g++){
			for(int i=0;i<m;i++){
				BF[ver][g][i]=0;
			}
		}
	}
    
	//Generate Bloom filter indexes selected by each vehicle
    for(int i=0; i<Total_vehicles; i++){ //Find BF-indexes for each vehicle 
    	Find_BF_Index(v_ids[i], v_BF_idx);
    }

    //vehicles (one by one) are entered into highway. add vehcile to corresponding start-receiver
    gettimeofday(&sim_stime,NULL);//Note start time of simulation
    ver=0;
    for(int i=0; i<Total_vehicles; i++){
    	BF[ver][v_starts[i]][v_BF_idx[i][0]]=1;
    	BF[ver][v_starts[i]][v_BF_idx[i][1]]=1;
    	BF[ver][v_starts[i]][v_BF_idx[i][2]]=1;
    	BF[ver][v_starts[i]][v_BF_idx[i][3]]=1;
    	v_status[i]=true; //vehicle is entered to the highway
    	gettimeofday(&v_stime[i],NULL);

    	usleep(60000);//sleep for 60 ms usleep(microsecond)
    }

    //Monitoring Location and adding to the passing Receiver
    int r=0; 
    while(r<=4){ // if Last vehcile rechaed to final receiver 4
    	sleep(4);
    	for(int i=0; i<Total_vehicles; i++){
    		if(v_status[i]==false) continue;
    		gettimeofday(&end,NULL);
			double t_time_ms=print_time(&v_stime[i], &end);
    		double dist=((t_time_ms)/(1000*60*60))*v_speeds[i]*1000;//Distnace in meters
    		if((int) dist%5000<100){ //If closer to a receiver
    			r=dist/5000;
    			int rec=r + v_starts[i];
    			if(rec<=v_ends[i]){
    				gettimeofday(&end,NULL);
    				if(print_time(&sim_stime, &end)>300005) {// if more than 5 miutes then initiate new version of BF
    					ver++;
    					gettimeofday(&sim_stime,NULL);
    				}
    				BF[ver][rec][v_BF_idx[i][0]]=1;
    				BF[ver][rec][v_BF_idx[i][1]]=1;
    				BF[ver][rec][v_BF_idx[i][2]]=1;
    				BF[ver][rec][v_BF_idx[i][3]]=1;
    				//std::cout<<"Vehicle : " << i <<" Distance : " << dist <<" Receiver : " << rec  <<"\n";
    			}
    			else
    			{
    				v_status[i]=false;

    			}
    			
    		}

    	}
    }




    //Results- With different time periods .. 
    std::ofstream r_file;
    r_file.open ("Simulation_Result.txt",std::ofstream::out | std::ofstream::app);

    for(int i=0; i<(n_r-1); i++) {
    	for(int j=i+1; j<n_r; j++){
    		r_file<< i <<","<<j<< ",";
    		for(int t=0; t<BF_versions; t++){
    			r_file<< estimate_flow(i,j,(int *)BF, t) << ",";
    		}
    		r_file<<"\n";
    	}
    }  
    r_file.close();
     return 0; 
}

bool Find_BF_Index(int id, int v_BF_idx[][k]){
	string str_id= to_string(id);
	char* key_begin;
	unsigned char hash_char[SHA256_DIGEST_LENGTH];
	std::size_t temp_length=0;
	unsigned long int idx=0;
	mpz_t mpz_temp;
	mpz_t size;
	int temp_int;
	mpz_init_set_ui (size, m);
    key_begin=(char*)str_id.c_str();
    temp_length=str_id.size();
    for(int l=0;l<k;l++){
      SHA256((unsigned char*)key_begin, temp_length, hash_char);
      mpz_inits(mpz_temp, NULL);
      mpz_import(mpz_temp, SHA256_DIGEST_LENGTH, 1, 1, 0, 0, hash_char);
      mpz_mod(mpz_temp, mpz_temp, size);
      idx=mpz_get_ui(mpz_temp);
      v_BF_idx[id][l]=idx;
      key_begin=(char*)hash_char;
      temp_length=SHA256_DIGEST_LENGTH;
    }
    return true;
}

double estimate_flow(int s_r,int e_r, int * BF, int t){

	if(t<(e_r-s_r)) return -1; // Not Possible to estimate 

	//Generate all RSUs Combinations
	int n_ar=e_r-s_r+1;
	int t_comb=pow(2,n_ar)-1;
	int comb[t_comb][n_ar];
	find_combinations(comb[0], t_comb, s_r, e_r);  	
		
	int t_Bf[m];
	

	double t_sum=0;

	int idx;
	for(int i=0; i<t_comb;i++){
		for(int p=0;p<m;p++)t_Bf[p]=0; //Initialize temporary BF to Zero
		for(idx=0; idx<n_ar; idx++){
			if(comb[i][idx]==-1)break;
			for(int v=t-(e_r-s_r); v<=t ; v++){
				//int bf_index=((v-(e_r-comb[i][idx]))* m * n_r) + (comb[i][idx]*m)+0;
				int bf_index= (v*m*n_r) + comb[i][idx] * m;
				Or_operation_ByteBF(t_Bf, &BF[bf_index],t_Bf, m);
			}
		}
		t_sum=t_sum+pow(-1, (idx+1))*estimate_no_of_element_in_1ByteBF(t_Bf);
		 
	}

  return t_sum; 
}

void Or_operation_ByteBF(int Bf1[], int Bf2[],int Bf3[], int s){
  for(int i=0; i<s; i++){
    if(Bf1[i]==0 && Bf2[i]==0)Bf3[i]=0;
    else Bf3[i]=1;
  }
}

double estimate_no_of_element_in_1ByteBF(int Bf[]){
  std::size_t bf_0s= Calculate_Zeros_in_ByteBf(Bf, m);
  double p= (double)bf_0s/m;
  long double nn=log(1-(double)1/m);
  double n=log(p)/(k*nn);
  return n;

}

std::size_t Calculate_Zeros_in_ByteBf(int Bf[], int s){
  std::size_t counter=0;
  for(int i=0; i<s; i++){
    if(Bf[i]==0)counter=counter+1; 
  }
  return counter;
}

void find_combinations(int *comb,int t_comb, int s_r,int e_r){
	int n_ar=e_r-s_r+1; 
	for(int i=0; i<t_comb; i++)for(int j=0; j<n_ar; j++)comb[i*n_ar+j]=-1;
	
	int c_comb=0;
	int receivers[n_ar];
	bool check[n_ar];
	int j=s_r;
	for(int i=0; i<n_ar; i++){
    receivers[i]=j; j++;
    check[i] = false;
  }

  for(int i=1; i<=n_ar; i++){
    recursion_find_combinations(receivers, i, 0, 0, check, n_ar, comb, &c_comb);
  }
  // for(int i=0; i<t_comb; i++){
  //   for(int j=0; j<n_r; j++){
  //     std::cout<<comb[i*n_r+j];
  //   }
  //   std::cout <<std::endl;
  // }
}

void recursion_find_combinations(int a[], int reqLen, int start, int currLen, bool check[], int len, int *comb, int *c_comb) 
{

  // Return if the currLen is more than the required length.
  if(currLen > reqLen)
  return;
  // If currLen is equal to required length then print the sequence.
  else if (currLen == reqLen) 
  {
    //std::cout<<"\t";
    int idx=0;
    for (int i = 0; i < len; i++) 
    {
      if (check[i] == true) 
      {
        comb[(*c_comb)*len+idx]= a[i];
        idx++;
        //std::cout<<a[i]<<" ";//BF updating
      }
    }
    *c_comb=*c_comb+1;
    //cout<<"\n";
    return;
  }
  // If start equals to len then return since no further element left.
  if (start == len) 
  {
    return;
  }
  // For every index we have two options.
  // First is, we select it, means put true in check[] and increment currLen and start.
  check[start] = true;
  recursion_find_combinations(a, reqLen, start + 1, currLen + 1, check, len, comb, c_comb);
  // Second is, we don't select it, means put false in check[] and only start incremented.
  check[start] = false;
  recursion_find_combinations(a, reqLen, start + 1, currLen, check, len, comb, c_comb);
}

double print_time(struct timeval *start, struct timeval *end) {
  double usec;
  usec = (end->tv_sec*1000000 + end->tv_usec) - (start->tv_sec*1000000 + start->tv_usec);
  return usec/1000.0;// time in ms
}