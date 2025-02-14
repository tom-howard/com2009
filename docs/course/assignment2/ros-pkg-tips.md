---  
title: Working with your ROS Package (in the Lab)
---  

Having followed [the instructions on the Getting Started page](./getting-started.md) in Week 1, your team's ROS package will be hosted on GitHub, which makes it much easier to collaborate, and transfer between simulation (e.g. WSL-ROS2, for example) and the real hardware in the lab. 

You'll need to transfer your ROS package to a robot laptop whenever you want to work on a real robot during the labs. 

## Working with Git and GitHub

You'll be working with Git and GitHub quite extensively throughout Assignment #2. Hopefully a lot of you will already be quite familiar with these tools, but if not, we would strongly recommend that you have a look at [This Course by the University of Sheffield's Research Software Engineering (RSE) Team](https://srse-git-github-zero2hero.netlify.app/){target="_blank"}.

## Setting Up SSH Keys

Using *SSH keys*, you can clone your team's ROS package to the robot laptops, make commits and push these back up to GitHub during the labs, without needing to provide your GitHub username and a personal access token every time. This makes life a lot easier! The following steps describe the process you should follow to achieve this (adapted from [GitHub Docs](https://docs.github.com/en/authentication/connecting-to-github-with-ssh){target="_blank"}).

### Step 0: Check if you already have an SSH Key on the Laptop

*These instructions are adapted from [this GitHub Docs page](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys?platform=linux){target="_blank"}.*

If you're generating an SSH key for the first time, then you can skip this step and go straight to the next section: "[Step 1: Generating an SSH key (on the Laptop)](#ssh-keygen)". If, however, you've already generated an SSH Key on the laptop previously then check it's still there before you go any further by following these steps...

1. Open a terminal on the laptop.

    ??? tip
        You can use the ++ctrl+alt+t++ keyboard combination to open a terminal!

2. Use the `ls` command as follows, to see if your SSH key already exists on the laptop:

    ```bash
    ls -al ~/.ssh
    ```

    This will provide you with a list of all the SSH keys on the laptop. Your team's key should have the same name as your ROS package ([if you followed the steps correctly when you created the key previously](#ssh-key-name)), and so you should see your key in the list, i.e.: `com2009_teamXX_2025.pub`.

3. If your key is there, then you're good to go... Either [clone your ROS package onto the Laptop](#ssh-clone) if you deleted it the last time you were in the lab, or just navigate back to it and pull down any updates (`git pull`) if you left it there.

    !!! warning
        We strongly recommend that you delete your team's package from the laptop at the end of each lab session.

4. If you can't see your key in the list, then you'll need to follow all the steps on this page, starting with [Step 1: Generating an SSH key (on the Laptop)](#ssh-keygen).

### Step 1: Generating an SSH key (on the Laptop) {#ssh-keygen}

1. From a terminal instance on the laptop navigate to the `~/.ssh` folder:

    ```bash
    cd ~/.ssh
    ```

1. Create a new SSH key on the laptop, using your GitHub email address:

    ```bash
    ssh-keygen -t ed25519 -C "your.email@sheffield.ac.uk"
    ```

    Replacing `your.email@sheffield.ac.uk` with **your GitHub email address**.

    <a name="ssh-key-name"></a>

1. You'll then be asked to **"Enter a file in which to save the key"**. This needs to be unique, so enter the name of your ROS package. For the purposes of this example, let's assume yours is called `com2009_team99_2025`.

1. You'll then be asked to **enter a passphrase**. This is how you make your SSH key secure, so that no other teams using the same laptop can access and make changes to your team's package/GitHub repo. You'll be asked to enter this whenever you try to commit/push new changes to your ROS package on GitHub. Decide on a passphrase and share this **ONLY** with your fellow team members. 

1. Next, start the laptop's ssh-agent:

    ```bash
    eval "$(ssh-agent -s)"
    ```

1. Add your SSH private key to the laptop's ssh-agent. You'll need to enter the name of the SSH key file that you created in the earlier step (e.g.: `com2009_team99_2025`)

    ```bash
    ssh-add ~/.ssh/com2009_team99_2025
    ```

    Replacing `com2009_team99_2025` with the name of your own SSH key file, of course!

1. Then, you'll need to add the SSH key to your account on GitHub...

### Step 2: Adding an SSH key to your GitHub account

*These instructions are replicated from [this GitHub Docs page](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?platform=linux){target="_blank"}*.

1. On the laptop, copy the SSH public key that you created in the previous steps to your clipboard.
    
    Do this from a terminal on the laptop, using `cat`:

    ```bash
    cat ~/.ssh/com2009_team99_2025.pub
    ```

    ...replacing `com2009_team99_2025` with the name of your SSH key file.

    The content of the file will then be displayed in the terminal... copy it from here.

    !!! tip "Tips"
        1. To copy text from inside a terminal window use ++ctrl+shift+c++
        2. You could also open the file in VS Code and copy it from there:

            ```bash
            code ~/.ssh/com2009_team99_2025.pub
            ```

2. Go to your GitHub account in a web browser. In the upper-right corner of any page, click your profile photo, then click **Settings**.

3. In the "Access" section of the sidebar, click **SSH and GPG keys**.

4. Click **New SSH key**.

5. Enter a descriptive name for the key in the "Title" field, e.g. `com2009_dia-laptop1`.

6. Select `Authentication Key` as the "Key Type."

7. Paste the text from your SSH Public Key file into the "Key" field.

8. Finally, click the "Add SSH Key" button.

## Cloning your ROS package onto the Laptop {#ssh-clone}

With your SSH keys all set up, you can now clone your ROS package onto the laptop. 

There's a ROS Workspace on each of the robot laptops and (much the same as in your own local ROS environment) your package **must** reside within this workspace!

1. From a terminal on the laptop, navigate to the ROS Workspace `src` directory:

    ```bash
    cd ~/ros2_ws/src/
    ```

1. Go to your ROS package on GitHub. Click the Code button and then select the SSH option to reveal the SSH address of your repo. Copy this. 

1. Head back to the terminal instance on the laptop to then clone your package into the `ros2_ws/src/` directory using `git`:

    ```bash
    git clone REMOTE_SSH_ADDRESS
    ```

    Where `REMOTE_SSH_ADDRESS` is the SSH address that you have just copied from GitHub.

1. Run Colcon to build your package, which is a **three-step process**:
	
    1. Navigate into the **root** of the ROS Workspace:

        ```bash
        cd ~/ros2_ws
        ```
    
    1. Run the `colcon build` command, targetting your package only:
    
        ```bash
        colcon build --packages-select com2009_team99_2025 --symlink-install
        ```
        (...again, replacing `com2009_team99_2025` with *your* team's package name.)
        
    1. Then, re-source your environment:
	
        ```bash
        source ~/.bashrc
        ```

1. Navigate into your package and run the following commands to set your identity, to allow you to make commits to your package repo:

    ``` { .bash .no-copy }
    cd com2009_team99_2025/
    ```
    ``` { .bash .no-copy }
    git config user.name "your name"
    ```
    ``` { .bash .no-copy }
    git config user.email "your email address"
    ```

You should then be able to commit and push any updates that you make to your ROS package while working on the laptop, back to your remote repository using the secret passphrase that you defined earlier!

## Deleting your ROS package after a lab session

Remember that the Robotics Laptops use an account that everyone in the class has access to. You might therefore want to delete your package from the laptop at the end of each lab session. It's very easy to clone it back onto the laptop again by following [the steps above](#ssh-clone) at the start of each lab session. Deleting your package (by following the instructions below) **won't** delete your SSH key from the laptop though, so you won't need to do all that again, and your SSH key will still be protected with the secret passphrase that you set up when generating the SSH Key to begin with (assuming that you are working on the same laptop, of course!) 

!!! warning
    Make sure you've pushed any changes to GitHub before deleting your package!

Delete your package by simply running the following command from any terminal on the laptop:

```bash
rm -rf ~/ros2_ws/src/com2009_teamXX_2025
```

... replacing `XX` with your own team's number!

## Returning in a Subsequent Lab Session

Your team will be provided with the same Robotics Laptop for each lab session. Having completed all the steps above in a previous lab session, you should be able to return to the laptop, re-clone your package and continue working with relative ease...

1. The private SSH key that you created in a previous lab session (and secured with a passphrase) should still be saved on the laptop. Check that this is the case by first running the following command:

    ```bash
    ls -al ~/.ssh
    ```
    If you can see your team's ssh key in the list then you're good to go. If not you'll need to [go back here and follow the steps to create it again](#ssh-keygen).

1. Next, start the laptop's ssh-agent and re-add your team's private key:

    ```bash
    eval "$(ssh-agent -s)"
    ```
    
    ``` { .bash .no-copy }
    ssh-add ~/.ssh/com2009_teamXX_2025
    ```

    Replacing `XX` with your team number.

1. Then, navigate to the ROS Workspace `src` directory:

    ```bash
    cd ~/ros2_ws/src
    ```

1. Clone your package into here, using the SSH address of your package on GitHub:

    ``` { .bash .no-copy }
    git clone REMOTE_SSH_ADDRESS
    ```

    You'll be asked for your secret passphrase, hopefully you remember it!
