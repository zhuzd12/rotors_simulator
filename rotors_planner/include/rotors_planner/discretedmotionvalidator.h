#ifndef OMPL_DISCRETED_MODEL_MOTION_VALIDATOR_
#define OMPL_DISCRETED_MODEL_MOTION_VALIDATOR_
 
 #include "ompl/base/MotionValidator.h"
 #include "ompl/base/SpaceInformation.h"


namespace ompl
{
    namespace base
    {
        class DiscretedMotionValidator : public MotionValidator
        {
        public:
            DiscretedMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }

            DiscretedMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }

            ~DiscretedMotionValidator() override = default;

            bool checkMotion(const State *s1, const State *s2) const override;

            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

            void setLongestValidSegment(double lvs) { longestValidSegment_ = lvs; }
            
        private:
            StateSpace *stateSpace_;

            double longestValidSegment_{0.2};

            void defaultSettings();
        };
    }
}

#endif
