#include "modelmotionvalidator.h"
#include "ompl/util/Exception.h"
#include <queue>

void ompl::base::ModelMotionValidator::defaultSettings()
{
    stateSpace_ = si_->getStateSpace().get();
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool ompl::base::ModelMotionValidator::checkMotion(const State *s1, const State *s2, std::vector<State *> &state_sequence) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    state_sequence.clear();
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    // bool result = true;
    int st_loop = 0;
    int st = 0;
    double dis_to_target_ = si_->distance(s1, s2);
    double accumulate_distance_ = 0.0;
    State *current_State = si_->allocState();
    State *next_State = si_->allocState();
    si_->copyState(current_State, s1);
    state_sequence.push_back(current_State);

    while(dis_to_target_ > path_deviation_ && st_loop < max_propagate_loops_)
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
            // State *new_State = si_->allocState();
            // si_->copyState(new_State, next_State);
            state_sequence.push_back(si_->allocState());
            si_->copyState(state_sequence[st], next_State);
            st++;
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

    lastValid.second = 1.0 - dis_to_target_/si_->distance(s1, s2);
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
        lastValid.second = 1.0 - dis_to_target_/si_->distance(s1, s2);
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



    // int nd = stateSpace_->validSegmentCount(s1, s2);

    // /* initialize the queue of test positions */
    // std::queue<std::pair<int, int>> pos;
    // if (nd >= 2)
    // {
    //     pos.push(std::make_pair(1, nd - 1));

    //     /* temporary storage for the checked state */
    //     State *test = si_->allocState();

    //     /* repeatedly subdivide the path segment in the middle (and check the middle) */
    //     while (!pos.empty())
    //     {
    //         std::pair<int, int> x = pos.front();

    //         int mid = (x.first + x.second) / 2;
    //         stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

    //         if (!si_->isValid(test))
    //         {
    //             result = false;
    //             break;
    //         }

    //         pos.pop();

    //         if (x.first < mid)
    //             pos.push(std::make_pair(x.first, mid - 1));
    //         if (x.second > mid)
    //             pos.push(std::make_pair(mid + 1, x.second));
    //     }

    //     si_->freeState(test);
    // }

    // if (result)
    //     valid_++;
    // else
    //     invalid_++;

    // return result;
}