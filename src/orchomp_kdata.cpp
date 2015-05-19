/** \file orchomp_kdata.cpp
 * \brief Implementation of orchomp_rdata, a parser for sphere data
 *        provided with an OpenRAVE kinbody XML file.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012-2013 Carnegie Mellon University */

/* This module (orchomp) is part of libcd.
 *
 * This module of libcd is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This module of libcd is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * A copy of the GNU General Public License is provided with libcd
 * (license-gpl.txt) and is also available at <http://www.gnu.org/licenses/>.
 */

#include <openrave/openrave.h>

#include "orchomp_kdata.h"

namespace orchomp
{

kdata::kdata() : OpenRAVE::XMLReadable("orchomp"){}

kdata::~kdata(){}

kdata_parser::kdata_parser(boost::shared_ptr<kdata> passed_d, const OpenRAVE::AttributesList& atts)
{
   /* save or construct the kdata object */
   this->d = passed_d;
   if(!this->d) this->d.reset(new kdata());
   /* get ready */
   this->inside_spheres = false;
   this->inside_ignorables = false;
}

OpenRAVE::XMLReadablePtr kdata_parser::GetReadable()
{
   return this->d;
}

OpenRAVE::BaseXMLReader::ProcessElement 
kdata_parser::startElement(const std::string& name, 
                           const OpenRAVE::AttributesList& atts)
{

   if (name == "ignorables"){
      if (this->inside_spheres) {
          RAVELOG_ERROR("you can't have <ignorables> inside <spheres>!\n");
      }
      if (this->inside_ignorables) {
          RAVELOG_ERROR("you can't have <ignorables> inside <ignorables>!\n");
      }

      this->inside_ignorables = true;
      return PE_Support;
   }

   else if (name == "ignore"){
      if (!this->inside_ignorables) {
          RAVELOG_ERROR("you can't have <ignore> "
                        "not inside <ignorables>!\n");
          return PE_Pass;
      }
      if (this->inside_spheres) {
          RAVELOG_ERROR("you can't have <ignore> "
                        "inside <spheres>!\n");
          return PE_Pass;
      }
      
      //increase the size of the vector,
      //    add an ignorable pair to the end.
      d->ignorables.resize( d->ignorables.size() + 1 );

      //iterate through the arguments, and assign things to the to
      for(OpenRAVE::AttributesList::const_iterator itatt = atts.begin();
          itatt != atts.end();
          ++itatt)
      {
         if (itatt->first=="link1"){
            d->ignorables.back().first = itatt->second;
         }else if (itatt->first=="link2"){
            d->ignorables.back().second = itatt->second;
         }else{
            RAVELOG_ERROR("unknown attribute %s=%s!\n",
                            itatt->first.c_str(),itatt->second.c_str());
         }
      }

      return PE_Support;
   }


   else if (name == "spheres")
   {
      if (this->inside_spheres) {
          RAVELOG_ERROR("you can't have <spheres> inside <spheres>!\n");
      }
      if (this->inside_ignorables) {
          RAVELOG_ERROR("you can't have <spheres> inside <ignorables>!\n");
      }

      this->inside_spheres = true;
      return PE_Support;
   }
   else if (name == "sphere")
   {

      if (!this->inside_spheres) {
          RAVELOG_ERROR("you can't have <sphere> not inside <spheres>!\n");
          return PE_Pass;
      }
      if (this->inside_ignorables) {
          RAVELOG_ERROR("you can't have <sphere> inside <ignorables>!\n");
          return PE_Pass;
      }
      
      //increase the size of the vector, add a sphere to the end.
      d->spheres.resize( d->spheres.size() + 1 );
      Sphere & current_sphere = d->spheres.back();

      //iterate through the arguments, and assign things to the to
      for(OpenRAVE::AttributesList::const_iterator itatt = atts.begin();
          itatt != atts.end();
          ++itatt)
      {
         if (itatt->first=="link"){
            current_sphere.linkname = itatt->second;
         }else if (itatt->first=="radius"){
            current_sphere.radius = strtod(itatt->second.c_str(), 0);
         }else if (itatt->first=="pos"){
            double pose[3];
            sscanf(itatt->second.c_str(), "%lf %lf %lf",    
                   &pose[0], &pose[1], &pose[2] );
            current_sphere.position = OpenRAVE::Vector( pose );
         }else if (itatt->first =="inactive"){
             if ( itatt->second =="true" ){
                current_sphere.inactive = true;
             }
         }else{
            RAVELOG_ERROR("unknown attribute %s=%s!\n",
                            itatt->first.c_str(),itatt->second.c_str());
         }
      }

      return PE_Support;
   }

   return PE_Pass;
}

void kdata_parser::characters(const std::string& ch)
{
   return;
}

bool kdata_parser::endElement(const std::string& name)
{
   if (name == "orchomp"){ return true; }
   
   if ( name == "ignorables" ){
      if (this->inside_spheres){
          RAVELOG_ERROR("you can't have </ignorables>"
                        " inside <spheres>!\n");
      }
      if (!this->inside_ignorables){
          RAVELOG_ERROR("you can't have </ignorables>"
                        " without matching <ignorables>!\n");
      }
      this->inside_ignorables = false;
   }
   else if (name == "ignore")
   {
      if (this->inside_spheres){
          RAVELOG_ERROR("you can't have </ignore> inside <spheres>!\n");
      }

      if (!this->inside_ignorables){
          RAVELOG_ERROR("you can't have </ignore> not"
                        " inside <ignorables>!\n");
      }
   }

   else if (name == "spheres")
   {
      if (this->inside_ignorables){
          RAVELOG_ERROR("you can't have </spheres> "
                        "inside <ignorables>!\n");
      }

      if (!this->inside_spheres){
          RAVELOG_ERROR("you can't have </spheres> "
                        "without matching <spheres>!\n");
      }
      this->inside_spheres = false;
   }
   else if (name == "sphere")
   {
      if (!this->inside_spheres){
          RAVELOG_ERROR("you can't have </sphere> inside <ignorables>!\n");
      }

      if (!this->inside_spheres){
          RAVELOG_ERROR("you can't have </sphere> not inside <spheres>!\n");
      }
   }
   else{
      RAVELOG_ERROR("unknown field %s\n", name.c_str());
   }
   return false;
}

} /* namespace orchomp */
