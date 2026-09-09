Reviewer 1

The paper presents a dataset of a single-segment tendon-driven continuum robot under several sensing modalities. The inclusion of pose and contact force information on an external object is interesting.
1. I consider that the paper needs a general review of how to present the developments achieved. Consider including diagrams, pictures, or flowcharts to help readers understand the process described. For example, Figure 1 is not clear what the author refers to by the contact wand; g-fix is not clear what it means or how it is used.
Maybe can include a Figure in the text: " The four actuator–sensor pairs are labeled 1–4, corresponding to tendons aligned with the x, y, −x, and −y directions"
2. The author mentions "Together, these design choices ensure that each recorded quantity has a clear physical correspondent in the governing equations of tendon-driven continuum robots". However, in the text it is not clear that the dataset fulfills any equations of tendon-driven continuum robots.
3. Using the sensor information as Opti-Track, kinematic information can be obtained; however, it is not clear how a forward dynamics simulation based on the Geometric Variable Strain (GVS) approach is used. What kind of information is obtained ?. It is convenient to compare the performance of the dynamic model with that of a single-segment tendon-driven continuum robot.

Reviewer 2

The manuscript entitled "Multi-sensor dataset of a tendon-driven continuum robot in dynamic motion" represents a very important contribution to the field. There is no doubt that such a dataset is missing and would accelerate the development of improved control models. Nevertheless, I have two concerns that might be addressed by the authors. Both of my concers revolve around the topic of embodied intelligence. While I agree that a dataset is missing to research and develop better control models, I also see the challenge of an intertwined hardward/software issue. Continuum robots, like many biological systems, are often designed for specific tasks, where the task guides the design.

First concern: In the introduction, the authors potentially over-state the need for a dataset in continuum robotics. Or at least, they neglect, the above mentioned, very important, aspect that make controlling such robots challenging: embodied intelligence or morphological computation. Unlike in the other fields, mentioned in the intro, the motion in the present case is inherently coupled to the construction material and hardware design of the robot. I suggest to rethink the paragraph maybe even talking about the contribution of hardware and environment to control.

Second concern: The present design was deliberately selected that the tendons do not contact the motor case for better measurement quantities. I agree, but I am wondering whether the whole robot design requires additional force sensors at the contact points of tendons and the inner surfaces of the equally spaced disks. By shortening one cable, the last disk is pulled towards the first disk and the cable makes contact to the outer part of the inner surface, where the cable is routed through. I suggest to comment on this fact and either explaining, why this measurement quantity is not needed or adding a statement for the limitations of the current design or dataset.

Additionally, I found one typo, the authors might want to correct.
Intro: please correct typo "between heterogeneous force measurements" instead of "between heterogeneous force measuraments"

Reviewer 3

General Comments
This paper presents a complete, well-validated multi-modal dataset for single-segment tendon-driven continuum robots, filling a clear gap in open benchmark resources for continuum robotics. The authors integrate six synchronized sensing modalities and split the dataset into quasi-static, dynamic trajectory, and contact interaction subsets, covering static identification, dynamic modeling, learning-based estimation and contact analysis. The experimental platform is fully documented with commercially available hardware, and both raw and uniformly processed data are shared on Figshare with open acquisition/post-processing code. Rigorous cross-modal validation including kinematic matching, force consistency checks, sensor noise characterization and temporal synchronization analysis thoroughly verifies dataset reliability for continuum robot. The manuscript is well-structured and technically sound, and only minor revisions are required before acceptance.
Here are some suggestions:
1.Add dedicated discussion about the generalization limitation of continuum robot datasets and corresponding improvement strategies to enhance the work’s broader impact:
It is widely acknowledged that continuum soft robots face an inherent trade-off between abundant compliant degrees of freedom and precise controllability, making data-driven modeling an indispensable solution. Compared with other soft robot variants, tendon-driven continuum robots benefit from relatively analytical kinematic frameworks, which partially mitigate this contradiction. Nevertheless, intrinsic material uncertainties such as viscoelastic creep, hysteresis and manufacturing deviations persistently degrade model transferability. Even for two structurally identical continuum robot prototypes trained on the same dataset, data-driven models often fail to generalize directly—new data collection and retraining are mandatory for each individual platform, severely limiting the reproducibility and reusability of open datasets and codes across laboratories. The authors are required to supplement targeted discussion of this critical challenge in the manuscript, and elaborate concrete approaches to improve the universality and cross-platform generalization of the presented dataset, platform and released codes for future research communities.
2.Revise figure captions to add brief explanatory notes for calibration sweeps and signal zero baselines for better standalone readability.

The dataset provides a valuable community benchmark analogous to KITTI and EuRoC for continuum robotics. All requested revisions are cosmetic and clarifying without impacting the core scientific contribution.

Reviewer 4

Review:

As this is my first review of a dataset paper, I used an LLM to define criteria:

1. Data Description & Scope
2. Data Quality & Validation
3. Metadata & Documentation
4. Accessibility & Reusability (FAIR Principles)
5. Technical Rigor & Reproducibility
6. Potential Impact & Use Cases
7. Ethical & Legal Considerations

I will structure my review accordingly:

1. Data Description & Scope

The scope of the paper is a dataset on a tendon driven continuum robot with the aim to provide validation data for computational models predicting the behaviour of soft robots in the interaction with the environment.
The introduction clearly states the purpose and nicely points out the value of dataset in other robotic disciplines highlighting the gap and need. A combination of static and dynamic scenarios and multi-modal data is a promising basis for developing methods for modelling of tendon-driven continuum robots.

The paper is very well written and comprehensive. From what I can judge, all relevant and even small details are described in the paper.


2. Data Quality & Validation
Raw and pre-processed data is provided. The Multi-modelity is very nice, as it allows details cross-validation between sensor modalities, which is performed already here in the paper. All of this looks very good.


3. Metadata & Documentation
The Dataset is well documented and in itself not super complicated. The Documentation in the paper is well done and should allow easy use of the dataset.


4. Accessibility & Reusability (FAIR Principles)
All data is accessible via figshare. I was able to download and inspect it and compare it to the paper description. It is well documented. A DOI is provided. Copyright: CC BY 4.0.


5. Technical Rigor & Reproducibility
Experimental setup and Data cross-validation are appropriate and technically sound.
Reproducibility is given, as the robot components are commercially available or 3D printed and not too difficult to assemble.

6. Potential Impact & Use Cases
The dataset is relevant for a specific type of continuum robot, but it is argued that this is "the most widely studied paradigm in continuum robotics". For researchers interested in modelling this type of actuation, this dataset will be very valuable.


7. Ethical & Legal Considerations
None



One points for a minor revision:

The type of robot is justified in the introduction briefly (commonly used continuum robot architecture) but I think this could be strengthened by a brief section on related robots and modelling attempts.

I assume that in the force contact experiments, the bending sensors and optical MoCap will differ more as this may introduce additional subsegmental deformations? Did you check this? This would be interesting to mention in the technical validation.

CC-BY 4.0 is specified on the figshare page, but not in the paper. This should be added for clarity. The code license is given (MIT)

2.2.4 first sentence is a bit unclear to me. "pose of selected..." Maybe: position and orientation of the robot discs (Ref to Fig 1)

If there are videos available from the recordings, I would encourage you to share them too. This could be very helpful.

!!!!! Disclaimer !!!! LLM generated content below. !!!!!!!
Once I read the paper and wrote this review, I asked the LLM (, hosted at academiccloud.de under DSGVO data privacy) to check my review and flag if I missed something critical.
This was the answer (shortened by me):


## 🔍 **Critical Points You May Have Missed (But Are Worth Noting)**

While your review is already strong, here are **a few subtle but important aspects** that *Nature Scientific Data* often emphasizes:

### 2. **Data Versioning & Updates**
> “Is this dataset versioned? Will future updates (e.g., new trajectories, sensor calibrations) be tracked via a DOI or version tag? If not, consider adding a note about long-term maintenance.”

👉 *Why this matters:* Journals like *Nature Scientific Data* value **long-term accessibility** and **version control**.

### 3. **Use of the Dataset in the Paper**
> “The paper uses the dataset to validate a model (GVS), but this is not a primary contribution. Consider adding a sentence in the ‘Usage Notes’ section: *‘This dataset has already been used to validate a forward dynamics model (GVS) in [citation], demonstrating its utility for benchmarking.’*”

👉 *Why this matters:* Shows **real-world impact** and reuse potential.

...

This is a high-quality, well-documented dataset paper that addresses a critical gap in continuum robotics. The multi-modal, synchronized data, rigorous cross-validation, and clear organization make it a valuable resource for modeling, control, and learning research. With minor revisions—particularly to clarify the robot’s context and licensing—the paper will be an excellent candidate for publication in *Nature Scientific Data

Reviewer 5

Attachment(s):

    Download Reviewer 5 attachment 1

