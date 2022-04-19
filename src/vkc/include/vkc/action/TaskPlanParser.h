#ifndef VKC_TASK_PLAN_PARSER_H
#define VKC_TASK_PLAN_PARSER_H

#include <ros/ros.h>
#include <stdlib.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <fstream>  // read and parse pddl plan file
#include <iostream>
#include <string>
#include <vector>

#include "ActionCreatorFactory.h"
#include "actions.h"

namespace vkc {
class TaskPlanParser {
 public:
  TaskPlanParser(SceneObjects &scene_objs) : act_creatory_(scene_objs) {}
  ~TaskPlanParser() {}

  void Parse(ActionSeq &actions, const std::string &file) {
    std::string line;
    char c_line[1024];  // we assume a line has no more than 1024 characters,
                        // this maybe the most common case
    std::ifstream ifs(file);

    while (ifs.good()) {
      ifs.getline(c_line, sizeof(c_line));
      line = c_line;
      ActionBase::Ptr p_action = ParseActionSymbol(line);
      if (p_action) {
        actions.emplace_back(p_action);
      } else {
        fprintf(stdout, "invalid action object while parsing action text: %s\n",
                line.c_str());
      }
    }

    ROS_DEBUG("the task plan file contains %lu actions", actions.size());
    ROS_DEBUG("******************************************\n\n\n");
  }

 private:
  void SplitString(std::vector<std::string> &fields, const std::string &str,
                   const std::string &seperator) {
    boost::split(fields, str, boost::is_any_of(seperator),
                 boost::token_compress_on);
  }

  ActionBase::Ptr ParseActionSymbol(const std::string &line) {
    auto start_pos =
        line.find("(") != std::string::npos ? line.find("(") + 1 : 0;
    auto end_pos =
        line.find(")") != std::string::npos ? line.find(")") : line.length();
    auto substr_len = end_pos - start_pos;
    fprintf(stdout, "[%s]line text: %s\n", __func__,
            line.substr(start_pos, substr_len).c_str());

    std::vector<std::string> fields;
    SplitString(fields, line.substr(start_pos, substr_len), " ");
    std::copy(fields.begin(), fields.end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout << std::endl << std::endl;

    return act_creatory_.CreateAction(fields);
  }

  ActionCreatorFactory act_creatory_;
};
}  // namespace vkc

#endif  // VKC_TASK_PLAN_PARSER_H
