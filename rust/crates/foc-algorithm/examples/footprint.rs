use core::mem::size_of;

use foc_algorithm::{
    AdaptiveSmoState, AdaptiveState, AdrcFastTdState, AdrcState, CascadeState, EkfFocState,
    EkfState, FocBasicState, FuzzyState, HfBemfState, HfInjectionState, HigherOrderSmoState,
    KalmanState, LuenbergerState, MpcState, NeuralNetworkState, PulseInjectionState,
    ReinforcementLearningState, RotatingHfState, SuperTwistingSmoState, UkfState,
};

fn print_size<T>(name: &str) {
    println!("{name}={}", size_of::<T>());
}

fn main() {
    print_size::<FocBasicState>("FocBasicState");
    print_size::<CascadeState>("CascadeState");
    print_size::<AdaptiveSmoState>("AdaptiveSmoState");
    print_size::<HigherOrderSmoState>("HigherOrderSmoState");
    print_size::<SuperTwistingSmoState>("SuperTwistingSmoState");
    print_size::<HfInjectionState>("HfInjectionState");
    print_size::<RotatingHfState>("RotatingHfState");
    print_size::<PulseInjectionState>("PulseInjectionState");
    print_size::<HfBemfState>("HfBemfState");
    print_size::<KalmanState>("KalmanState");
    print_size::<LuenbergerState>("LuenbergerState");
    print_size::<EkfState>("EkfState");
    print_size::<EkfFocState>("EkfFocState");
    print_size::<UkfState>("UkfState");
    print_size::<AdrcState>("AdrcState");
    print_size::<AdrcFastTdState>("AdrcFastTdState");
    print_size::<AdaptiveState>("AdaptiveState");
    print_size::<FuzzyState>("FuzzyState");
    print_size::<MpcState>("MpcState");
    print_size::<NeuralNetworkState>("NeuralNetworkState");
    print_size::<ReinforcementLearningState>("ReinforcementLearningState");
}
