# Multi-Effects-Plugin

This project was completed at the University of Miami in Will Pirkle's audio plugin design class as my final project.

It is a real-time DSP audio plugin developed in RackAFX using C++. 

The Tank Bank Multi FX is a multi effects processor plugin that consists of an envelope follower, a delay, two modulated delays, a reverb and a tremolo effect. It is a versatile effects unit that can create a wide variety of sounds and be used for many applications.


 
Design
 
​The Tank Bank gives the user control over several parameters of each of the effects and allows them to switch between the 4 main options. Several of the effects have their own set of controls and options, like choosing between the standard delay or one of the modulated delays.
 
 
 
Envelope Follower
 
​The envelope follower takes the input audio and creates an output that is the envelope of the original signal. There are several added parameters to the effect. The Pre gain controls the level of the original signal in the loop. The envelope detector then has its own set of parameters for the attack and release. Next, the values of the threshold and cutoff frequency are calculated and set by the user in the GUI. The couple Q is a switch that sets the cutoff frequency to control the value of Q when it is activated. The signal then passes through the low pass filter stage, which is controlled by the four main BiQuad structures implemented. Their is another added feature of the moog gain compensation, which when activated decreases the overall frequencies of the signal as the Q is increased.
 
 
 
Delay + Modulated Delays
 
Delay with HPF in Feedback
The Delay Module of this plugin uses the model of a standard analog delay, with a high pass filter in the feedback loop. This allows the user to filter the wet feedback signal, to have only frequencies above the cutoff, which can be useful in reducing muddiness that typically comes with modulating low frequencies.
 
 
 
Flanger
 
The flanger module of this plugin is just a standard flanger. I added this option to the plugin originally to modify and make it the modulated delay section of the project, but after I had already added the code for the basic effect I decided to work on a different form of modulated delay, so I left the flanger in anyways for more user options. The user can change the rate, depth, and resonance of the signal to create the desired effect. The user can also select between a sine and triangle shaped waveform to modulate the signal.
 
 
 
Bass Chorus
 
The Bass Chorus is the main modulated delay section of the Sick Effects plugin project. The input signal is split into a low passed and high passed signal using Linkwitz-Riley filters, which have an attenuation of -6dB at the cutoff frequency instead of the usual – 3dB attenuation in most filters. The low passed filter signal is sent as a “dry” loop to the summation at the output, while the high passed filter signal is modulated by the chorus. The chorus section gives the user a choice of LFO type, as well as the standard chorus controls depth, rate, and resonance. The low passed and high passed signals are then summed together, and when summed at the same cutoff frequency the resulting response is flat. I added a knob to control to cutoff of the low pass filter, which is independent of the high pass filter cutoff, meaning that the user can modify the low pass cutoff frequency via the knob in the user interface while the high passed portion is always at 1000Hz.
 

 
Reverb
 
​The reverb section of the plugin uses a reverb tank and several filters to implement a digital reverb, there is no convolution or any impulse responses. The signal first passes through the tank, which is a series of all pass filters and delays (pictured on the next page) that created the reverb effect. The length of all pass filters and delays are variable in RackAFX but not to the user in the plugin GUI as that complicates it beyond need. However, several different types of reverbs can be created when changing the values in RackAFX. The signal then passes through two damping low pass filters, which help to reduce the high frequencies in the reverb that sound like harsh reflections against concrete surfaces and can deteriorate the quality of the sound. The user is given control over the wet/dry mix and the krt reverb time length values in the GUI, keeping the plugin simple and effective.
 

 
Tremolo
 
​The tremolo stage allows the user to switch between a tremolo and a panner effect. The tremolo is standard and functions by creating a consistent varying the amplitude of the signal. The user can control the rate (how fast the amplitude changes) and depth (how much attenuation of the signal) of the effect in the GUI. If the user switches over to the panner effect, the signal is then panned between the left and right stereo outputs in the same fashion of the tremolo. The rate controls how fast the signal pans between each side and the depth controls how much of the original signal is separated.
 
 
 
