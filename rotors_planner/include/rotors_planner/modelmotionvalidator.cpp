#include "modelmotionvalidator.h"
#include "ompl/util/Exception.h"
#include <queue>

void ompl::base::ModelMotionValidator::defaultSettings()
{
    stateSpace_ = si_->getStateSpace().get();
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool ompl::base::ModelMotionValidator::checkMotion(const State *s1, const State *s2, std::vector<State *> &state_sequence, std::vector<double>& time_stamps) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    // bool result = true;
    int st_loop = 0;
    double dis_to_target_ = si_->distance(s1, s2);
    double accumulate_distance_ = 0.0;
    int segment_loops = 0;
    int segments = 0;
    State *current_State = si_->allocState();
    State *next_State = si_->allocState();
    si_->copyState(current_State, s1);

    while(dis_to_target_ > path_resolution_ && st_loop < max_propagate_loops_)
    {
        Controlfn_(current_State, s2, duration_, next_State);

        accumulate_distance_ = accumulate_distance_ + si_->distance(current_State, next_State);
        segment_loops++;
        dis_to_target_ = si_->distance(current_State, s2);
        if(accumulate_distance_ >= path_resolution_ || dis_to_target_ < path_deviation_)
        {
            if(!si_->isValid(next_State))
            {
                si_->freeState(next_State);
                si_->freeState(current_State);
                si_->freeStates(state_sequence);
                OMPL_DEBUG("shit");
                return false;
            }
            state_sequence.push_back(si_->allocState());
            si_->copyState(state_sequence[segments], next_State);
            time_stamps.push_back(double(segment_loops)*duration_);
            accumulate_distance_ = 0.0;
            segment_loops = 0;
            segments++;
        }

        // state_sequence.push_back(si_->allocState());
        // si_->copyState(state_sequence[st_loop], next_State);
        // time_stamps.push_back(double(st_loop+1)*duration_);

        si_->copyState(current_State, next_State);
        st_loop ++;
    }

    si_->freeState(next_State);
    si_->freeState(current_State);
    if(st_loop >= max_propagate_loops_)
        return false;
        
    return true;

}

bool ompl::base::ModelMotionValidator::checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    // bool result = true;
    int st_loop = 0;
    double dis_to_target_ = si_->distance(s1, s2);
    double accumulate_distance_ = 0.0;
    State *current_State = si_->allocState();
    State *next_State = si_->allocState();
    si_->copyState(current_State, s1);

    // lastValid.second = 1.0 - dis_to_target_/si_->distance(s1, s2);
    lastValid.second = 0.0; // represent time to go
    if (lastValid.first != nullptr)
        si_->copyState(lastValid.first, current_State);

    while(dis_to_target_ > path_resolution_ && st_loop < max_propagate_loops_)
    {
        Controlfn_(current_State, s2, duration_, next_State);

        accumulate_distance_ = accumulate_distance_ + si_->distance(current_State, next_State);
        dis_to_target_ = si_->distance(current_State, s2);
        if(accumulate_distance_ >= path_resolution_ || dis_to_target_ < path_deviation_)
        {
            if(!si_->isValid(next_State))
            {
                si_->freeState(next_State);
                si_->freeState(current_State);
                return false;
            }
            accumulate_distance_ = 0.0;
        }
        // lastValid.second = 1.0 - dis_to_target_/si_->distance(s1, s2);
        lastValid.second += duration_;
        if (lastValid.first != nullptr)
            si_->copyState(lastValid.first, next_State);

        si_->copyState(current_State, next_State);
        st_loop ++;
    }

    si_->freeState(next_State);
    si_->freeState(current_State);
    if(st_loop >= max_propagate_loops_)
        return false;
        
    return true;

}

bool ompl::base::ModelMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    double accumulate_time = 0.0;

    // bool result = true;
    int st_loop = 0;
    double dis_to_target_ = si_->distance(s1, s2);
    double accumulate_distance_ = 0.0;
    State *current_State = si_->allocState();
    State *next_State = si_->allocState();
    si_->copyState(current_State, s1);

    while(dis_to_target_ > path_resolution_ && st_loop < max_propagate_loops_)
    {
        Controlfn_(current_State, s2, duration_, next_State);
        accumulate_time += duration_;
        accumulate_distance_ = accumulate_distance_ + si_->distance(current_State, next_State);
        dis_to_target_ = si_->distance(current_State, s2);
        if(accumulate_distance_ >= path_resolution_ || dis_to_target_ < path_deviation_)
        {
            if(!si_->isValid(next_State))
            {
                si_->freeState(next_State);
                si_->freeState(current_State);
                return false;
            }
            accumulate_distance_ = 0.0;
        }

        si_->copyState(current_State, next_State);
        st_loop ++;
    }

    si_->freeState(next_State);
    si_->freeState(current_State);
    if(st_loop >= max_propagate_loops_)
        return false;
        
    return true;
}