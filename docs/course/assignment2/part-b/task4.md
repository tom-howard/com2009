---  
title: "Task 4: Documentation"  
---  

## Summary

For this task you will write some documentation to describe the application that you have developed **[for Task 3](./task3.md)**.

Your documentation should be contained within a single `README.md` file which must sit in the root of your team's package directory, i.e.: 

``` { .txt .no-copy }
com2009_teamXX_2025/README.md
``` 

!!! warning "Important"
    Your documentation file **MUST** be called `README.md`, it **MUST** be in the root of your package directory, and it must be created with *Markdown Formatted Text* ([see below](#formatting)).

## Content of your README

Your documentation should contain the following information.

### Overview

A brief explanation of what your application does (no more than 100 words).

### Installation and Execution

Explain (to someone who may not already be familiar) how to install and execute your package on one of the Robotics Laptops in the lab.

#### Exceptions

1. In your documentation you can assume that [Steps 1-4 of the Robot/Laptop Setup process](../../../waffles/launching-ros.md) have already been carried out, so **you don't need to discuss any of this**.
1. You can also assume (for the purposes of this documentation) that your package is *public*, so there's no need to include any information about creating ssh keys etc (assume anyone could download your package through a simple `git clone ...`).

#### Dependencies

This section should also provide details on all the external packages that your application depends upon in order to function (i.e. any Python/ROS 2 libraries that you are using that exist outside your own package). 

### Functional Description

Explain how your application works. This will form the bulk of the documentation, and should include a *Functional Block Diagram (FBD)* (or multiple FBDs if you wish) to aid the explanation and illustrate the control logic. This should be more than simply a ROS node/topic graph. 

[See here for information on how to include images in your README.md](https://docs.github.com/en/get-started/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax#images){target="_blank"} to ensure that they are rendered correctly (see the information on using *relative links* to images that exist within your repository).

### Contributors

List all contributing team members and provide links to their GitHub profiles.

## Word Count

Your documentation should be **600-800 words** in length, any content after the 800-word limit won't be read (and therefore won't be considered in the assessment either).

## Formatting

Your `README.md` file should be formatted using *GitHub Flavoured Markdown*. You can find the [basic formatting syntax here](https://docs.github.com/en/get-started/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax){target="_blank"}, which should be sufficient for the purposes of this task. If you really want to do more however, then [the full GitHub Flavoured Markdown Specification can be found here](https://github.github.com/gfm/){target="_blank"}.

When marking this task we will view your `README.md` file on GitHub, so it's important that you check the formatting yourself prior to the Part B deadline. **You will lose marks for formatting errors**!

## Marking 

There are **20 marks** available for this task in total, distributed as follows.

<center>

| Criteria | Marks | Details |
| :--- | :---: | :--- |
| **A**: Overview | 5/20 | A clear and concise summary of the application, and a full and correct explanation of how to install and execute it. |
| **B**: Functional Description | 10/20 | A detailed and accurate description of how the application works (or *was intended to work[^task3-perfomance]*). A clear yet detailed Functional Block Diagram should be included to support the discussion. |
| **C**: Formatting and Writing Standard | 5/20 | Clear, concise and professional writing throughout, which uses technical language appropriately but which is accessible to a none experienced reader. The `README.md` file must be correctly formatted, i.e. headings, styling, code blocks, other text formatting, figures etc. should all be rendered correctly when viewed directly on GitHub. |

</center>

[^task3-perfomance]: The marking of this task is *not dependent* on your team's performance in Task 3, so even if you score 0/40 marks in Task 3, you could still score 20/20 marks for this task if you document your work well.
