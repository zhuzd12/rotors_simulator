 #ifndef OMPL_BASE_MODEL_MOTION_VALIDATOR_
 #define OMPL_BASE_MODEL_MOTION_VALIDATOR_
 
 #include "ompl/base/MotionValidator.h"
 #include "ompl/base/SpaceInformation.h"


namespace ompl
{
    namespace base
    {
        typedef std::function<bool(const State *, const State *, const double, State *)> ControllerFn;

        class ModelMotionValidator : public MotionValidator
        {
        public:
            ModelMotionValidator(SpaceInformation *si, double pr, double mpl, double du, const ControllerFn &svc) : MotionValidator(si),
                path_resolution_(pr), max_propagate_loops_(mpl), duration_(du), Controlfn_(svc)
            {
                defaultSettings();
            }

            ModelMotionValidator(const SpaceInformationPtr &si, double pr, double mpl, double du, const ControllerFn &svc) : MotionValidator(si),
                path_resolution_(pr), max_propagate_loops_(mpl), duration_(du), Controlfn_(svc)
            {
                defaultSettings();
            }

            void setController(const ControllerFn &svc)
            {
                Controlfn_ = svc;
            }

            const ControllerFn& getController() const
            {
                return Controlfn_;
            }

            ~ModelMotionValidator() override = default;

            bool checkMotion(const State *s1, const State *s2) const override;

            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

            bool checkMotion(const State *s1, const State *s2, std::vector<State *> &state_sequence) const;

        private:
            StateSpace *stateSpace_;
            ControllerFn Controlfn_;
            double path_resolution_{0.5};
            double max_propagate_loops_{200};
            double path_deviation_{0.2};
            double duration_{0.02};

            void defaultSettings();
        };
    }
}

#endif