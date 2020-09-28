#include <string>
#include <vector>
#include <iostream>
class Solution {
public:
    int max_num = 0;
    string longestPalindrome(string s) {
        std::string out("");
        
        std::vector<int> sym = {}; // even
        std::vector<int> asym = {}; // odd
        if(s.length() > 0){
            for (int i = 0; i < s.length()-1; ++i){
                if(s.at(i) == s.at(i+1)){
                    sym.push_back(i);
                    // std::cout << "sym = " << i << std::endl;
                }
                if(i < s.length()-2){
                    if(s.at(i) == s.at(i+2)){
                        asym.push_back(i+1);
                        // std::cout << "asym = " << i+1 << std::endl;
                    }
                }
            }

            for(int i = 0; i < sym.size(); ++i){  // even
                string s1 = s.substr(0, sym.at(i)+1);
                string s2 = s.substr(sym.at(i)+1, s.length()-sym.at(i)-1);
                // if (s1.length() >= s2.length() ){
                //     string s3 = s1;
                //     s1 = s2;
                //     s2 = s3;
                // }
                // std::cout << "sym " << s1 << " " << s2 << std::endl;
                string temp_out = longest_str(s1, s2, "");
                // std::cout << temp_out << std::endl;
                if(temp_out.length() > out.length()){
                    out = temp_out;
                }
            }

            for(int i = 0; i < asym.size(); ++i){  // odd
                string s1 = s.substr(0, asym.at(i));
                string s2 = s.substr(asym.at(i)+1, s.length()-asym.at(i)+1);
                //  if (s1.length() >= s2.length() ){
                //     string s3 = s1;
                //     s1 = s2;
                //     s2 = s3;
                // }
                // std::cout << "asym " << s1 << " " << s2 << std::endl;
                string temp_out = longest_str(s1, s2, s.substr(asym.at(i), 1));
                // std::cout << temp_out << std::endl;
                if(temp_out.length() > out.length()){
                    out = temp_out;
                }
            }
            if(out == ""){
                out = s.at(0);
            }
        }
        return out;
    }
    
    string longest_str(string s1, string s2, string con){
        string out = "";
        string s3, s4;
        // int local_max = 0;
        int s1_it = s1.length()-1;
        int s2_it = 0;
        for(int j = min(s1.length()-1, s2.length()-1) ; j >= 0 ; --j){
            if(s1.at(s1_it) == s2.at(s2_it) ){
                // local_max++; 
                s3 = s1.substr(s1_it, s1.length()-s1_it);
                s4 = s2.substr(0, s2_it+1);
                // std::cout << "  " << s3 << "  " << s4 << std::endl;
            }
            else{
                break;
            }
            s2_it++;
            s1_it--;
        }
        // if(local_max > max_num){
            // max_num = local_max;
            out = s3 + con + s4;
        // }
        // out = ;
        return out;
    }
};

//  std::string s1 = s.substr(0,i);
//             std::string s2 = s.substr(i, s.length()-i);
//             string s3, s4;
//             int local_max = 0;    
//             std::cout << s1 << "  " << s2 << std::endl;
//             int k = 0;
//             for(int j = s1.length()-1; j >=0 ; --j){
//                 if(s1.at(j) == s2.at(k) ){
//                     local_max++; 
//                     s3 = s1.substr(j, s1.length()-1);
//                     s4 = s2.substr(k, s2.length()-1);
//                     std::cout << "  " << s3 << "  " << s4 << std::endl;
//                 }
//                 else if(s2.length > 1){
//                     if ((s1.at(j) == s2.at(k+1)){
//                         local_max++; 
//                         s3 = s1.substr(j, s1.length()-1);
//                         s4 = s2.substr(k+1, s2.length()-1);
//                         std::cout << " sec " << s3 << "  " << s4 << std::endl;
//                     }
//                 }
                    
//                 }
//                 k++;
//             }
//             if(local_max > max_num){
//                 max_num = local_max;
//                 out = s3+s4;
//             }
