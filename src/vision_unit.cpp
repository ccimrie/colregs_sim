#include <vision_unit.h>
#include <iostream>
#include <map>
#include <yaml-cpp/yaml.h>
#include <string_view>
#include <dirent.h>
#include<filesystem> // C++17 feature
#include <sys/types.h>
#include <csv_functions.h>

using namespace std;

VisionUnit::VisionUnit()
{
  // Empty constructor
}

VisionUnit::VisionUnit(std::string yaml_config_file)
{
  printf("Instantiating vision unit...\n");
  YAML::Node config = YAML::LoadFile(yaml_config_file);
  string distance_info_filename=config["ship distance csv"].as<string>();
  string confusion_matrix_info_filename=config["confusion matrix csv dir"].as<string>();

  // // Extract ship distance
  vector<pair<string, vector<string>>> distance_info=readCSV(distance_info_filename);
  cout << "Placing into map" << endl;
  int no_vessel_types=distance_info[0].second.size();
  for(int i=0; i<no_vessel_types; ++i)
  {
    // type, height, 640x480, 480x320, 320x240, 240x180
    vessel_class_info temp_info;
    temp_info.height=stod(distance_info[1].second[i].data());
    temp_info.res_640_480=stod(distance_info[2].second[i].data());
    temp_info.res_480_320=stod(distance_info[3].second[i].data());
    temp_info.res_320_240=stod(distance_info[4].second[i].data());
    temp_info.res_240_180=stod(distance_info[5].second[i].data());
    string v_type=distance_info[0].second[i].data();
    vessel_info[v_type]=temp_info;
  }
  // Extract confusion matrix and convert to probabilities
  for(const auto&entry:filesystem::directory_iterator(confusion_matrix_info_filename)) 
  {
    string filename=entry.path().filename();
    if (filename.find(".csv")!=string::npos)
    {
      cout << filename << endl;
      vector<pair<string, vector<string>>> conf_matrix=readCSV(confusion_matrix_info_filename+"/"+filename);
     
     // Remove .csv extension
      size_t extension_pos=filename.find(".csv");
      filename=filename.substr(0,extension_pos); 
      vector<string> tokens;
      size_t pos = 0;
      std::string token;
      string delimiter="_";
      while ((pos = filename.find(delimiter)) != std::string::npos)
      {
          token = filename.substr(0, pos);
          tokens.push_back(token);
          filename.erase(0, pos + delimiter.length());
      }
      tokens.push_back(filename);

      string dist_mat=tokens[2];
      printf("added tokens\n");

     // Get probability values
      for(int i=1; i<no_vessel_types; ++i)
      {
        double sum=0;
        string v_type=conf_matrix[0].second[i].data();
        for (int j=1; j<conf_matrix.size(); ++j)
        {
          string pred_v_type=conf_matrix[j].first.data();
          double val=stod(conf_matrix[j].second[i]);
          sum+=val;
          vessel_info[v_type].p_detection[dist_mat][pred_v_type]=val;
        }
        for (map<string,double>::iterator it_norm=vessel_info[v_type].p_detection[dist_mat].begin(); it_norm!=vessel_info[v_type].p_detection[dist_mat].end(); ++it_norm)
        {
          vessel_info[v_type].p_detection[dist_mat][it_norm->first]/=sum;
        }
      }
    }
  }
}

string VisionUnit::predict(string true_class, double distance)
{
 // Get distance discretisation
  double upper_dist_lim=1.5;
  string dist_res;
  if(distance<vessel_info[true_class].res_640_480*upper_dist_lim) dist_res="480x640"; 
  else if(distance<vessel_info[true_class].res_480_320*upper_dist_lim) dist_res="320x480";
  else if(distance<vessel_info[true_class].res_320_240*upper_dist_lim) dist_res="240x320";
  else if(distance<vessel_info[true_class].res_240_180*upper_dist_lim) dist_res="180x240";
 
 // Get probabilities
  double p_sum=0.0;
  double p_choice=0.0;
  map<string, double>::iterator it;
  string predicted_class;
  for (it=vessel_info[true_class].p_detection[dist_res].begin(); it!=vessel_info[true_class].p_detection[dist_res].end(); ++it)
  {
    if (p_choice<it->second+p_sum)
    {
      predicted_class=it->first;
      break;
    }
    else p_sum+=it->second;
  }
  return predicted_class;
}