/*
Created on Thurs June 19 16:42 2019

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2019, Dobot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu


#include <magician_hardware/magician_device.h>

namespace magician_hardware {

MagicianDevice::MagicianDevice()
{
    double default_joint_offsets[4]={0, 0, 0, 0};
    joint_offsets_=std::vector<double>(default_joint_offsets, default_joint_offsets+4);
    joint_bases_=std::vector<double>(default_joint_offsets, default_joint_offsets+4);
}

MagicianDevice::~MagicianDevice()
{

}

bool MagicianDevice::InitPose()
{
    Pose pose;
    int get_pose_times=0;
    int result=DobotCommunicate_InvalidParams;
    while (result!=DobotCommunicate_NoError) {
        result=GetPose(&pose);
        get_pose_times++;
        if(get_pose_times>5)
        {
            return false;
        }
    }

    for(size_t i=0; i<joint_bases_.size(); i++)
    {
        joint_bases_[i]=pose.jointAngle[i]*RAD_PER_DEGREE;
    }

    return true;
}

bool MagicianDevice::ReadPose(std::vector<double> &joint_values)
{
    Pose pose;
    int result=GetPose(&pose);
    bool pose_changed=false;
    if(result==DobotCommunicate_NoError)
    {
        double offset=0;
        for(size_t i=0; i<joint_bases_.size(); i++)
        {
            offset+=fabs(joint_bases_[i]-pose.jointAngle[i]*RAD_PER_DEGREE);
        }

        if(offset>1*RAD_PER_DEGREE)
        {
            pose_changed=true;
        }
    }

    if(pose_changed)
    {
        for(size_t i=0; i<joint_bases_.size(); i++)
        {
            joint_bases_[i]=pose.jointAngle[i]*RAD_PER_DEGREE;
        }
        double default_joint_offsets[4]={0, 0, 0, 0};
        joint_offsets_=std::vector<double>(default_joint_offsets, default_joint_offsets+4);
    }

    joint_values.resize(joint_bases_.size());
    for(size_t i=0; i<joint_bases_.size(); i++)
    {
        joint_values[i]=joint_bases_[i]+joint_offsets_[i];
    }

    return true;
}

bool MagicianDevice::WritePose(const std::vector<double> &joint_values)
{
    assert(joint_values.size()==4);

    std::vector<double> pulse_angles=joint_values;
    std::vector<double> pulses;
    pulses.resize(pulse_angles.size());

    for (size_t i=0; i<pulse_angles.size(); i++)
    {
        pulse_angles[i]-=joint_offsets_[i]+joint_bases_[i];
        pulses[i]=round(pulse_angles[i]*PULSE_PER_RAD);
    }

    PluseCmd cmd;
    cmd.j1=pulses[0];
    cmd.j2=pulses[1];
    cmd.j3=pulses[2];
    cmd.j4=pulses[3];
    cmd.e1=0;
    cmd.e2=0;

    uint64_t index;

    int send_pulse_times=0;
    int result=DobotCommunicate_InvalidParams;

    while(result!= DobotCommunicate_NoError && send_pulse_times<2) {
        result=SendPluse(&cmd, true, &index);
        send_pulse_times++;
    }

    if(result==DobotConnect_NoError)
    {
        for (size_t i=0; i<joint_offsets_.size(); i++)
        {
            joint_offsets_[i]+=pulses[0]*RAD_PER_PULSE;
        }
        return true;
    }
    else
    {
        return false;
    }
}

}


int main(int argc, char** argv)
{
    return 0;
}
