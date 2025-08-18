# Command_Interpreter

---

# Developer Resources

## Prerequisites
This guide assumes that you have access to a Unix-like environment (i.e. Linux/WSL or MacOS). You can make this work in Windows, but it's not officially supported. If you'd like to get set up with WSL, see the **Setting up WSL** section later in this guide.

Installation for these prerequisites varies from system to system. You can check if you have these installed by typing `{application} --version` into the terminal and seeing if it lists a version, or if it throws an error instead. A good place to start is checking if these tools are offered through your package management system (like apt or snap).
- git
  - This is used for source code management, and is **not** the same as GitHub. Installing git allows you to use command-line tools to push and pull code from the GitHub repo.
- cmake
  - You need a version >= 3.28. This is used for build automation: it keeps track of the project and generates "Makefiles" to compile the code.
- gcc and g++
  - gcc and g++ are compilers for the C and C++ programming languages, respectively. They turn the program from a text form to machine code that can be read and executed by the processor. They generate the executable files (`.exe` in Windows) that can be run by the computer. If you install gcc this should also include g++, but it doesn't hurt to run `g++ --version` after installation to check, just in case.
- make
  - Not to be confused with CMake. This is used to **run** (not generate) Makefiles, which are files that give instructions to the compiler on how to compile the project's source files into one executable file. Makefiles list the dependencies that source files have on each other and what compilation commands should be run to successfully compile them.

## Using Git(Hub)

### Cloning the repo
1. Open the terminal and `cd` into the directory that you want to clone the project into.
2. Run the command `git clone https://github.com/Cyclone-Robosub/Command_Interpreter.git`

### Pushing Code
1. **Make sure the unit tests pass!** If your code isn't yet complete, consider making a new test branch.
    - See **"Making a New Branch"**.
2. In the terminal, navigate to the project root.
3. Run the command `git add {args}`, where `{args}` is every file that you changed and wish to push to the repo.
    - For example, if you changed the files `Command.hpp`, `Command_Interpreter.cpp`, and `Command_Interpreter.hpp`, your
   command could look like:
   ```bash
    git add Command.hpp Command_Interpreter.* 
   ```
4. Type a commit title on the first line in the text editor window that pops up. Keep this concise but meaningful.
5. Type a longer commit description two lines below this (hit enter twice) with more information on the commit, if needed.
6. Close the text editor.
   - Note: if you want to cancel the commit, delete your commit notes and then close the text editor.
7. Run the command `git push`
   - You may need to enter credentials here. If you don't have a Personal Access Token (PAT) configured, you will probably need to do so. You want a "classic" token. When prompted to authenticate, you'll type your username as your GitHub username and your password as your PAT (**not** your password!). More information here: https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens

### Pulling Code
1. In the terminal, navigate to the project directory.
2. Run the command `git pull`
   - Optionally, you can first run `git fetch` and confirm that you're sure you want to make the changes before pulling them.

### Making a New Branch
1. Pick a name for your branch.
2. In the terminal, navigate to the project directory.
3. Run the command `git branch {name}`, where `{name}` is the name of the branch.

### Switching Branches
1. In the terminal, navigate to the project directory.
2. Use the `switch {name}` command to switch between different branches.
   - For example, to switch to the branch "GTest", type `switch GTest`. 
   - Reminder: the default branch is called "master".

## Navigating the Project

### Project Structure
- Code that we write lives in `src/`.
- Code that we write for unit testing (to make sure our code is correct) lives in `testing/`.
- Other libraries that we use have their own directories as well, named correspondingly.
- You'll make your own `build/` folder (see **"Using CMake to Build Code"**)
- `CMakeLists.txt` and `README.md` (this file!) live on the same level as the directories listed above.

### Basic Terminal Commands
- `ls`: These are for Unix (Linux or macOS) and Windows respectively. These list all the files and folders in your current location, excluding hidden files.
  - To see hidden files (like .gitignore), you can run `ls -A`
- `cd {path}`: This means to **c**hange **d**irectory to the directory specified in `{path}`. Path can be either a "relative" path or an "absolute" path.
  - A relative path is a path defined from your current location. If you're used to a graphical file explorer, you can think of this as all the folders and files that you can see from within whatever folder you're currently in.
  - An absolute path is a path defined from the "root" (start) of the file system. On a Unix system, this looks like `/`. If you're normally a Windows user, you'll recognise it as `C:\ `.
  - There are a few "special" files: `.` means "the current directory" and `..` means "the directory one level above".
    - For example, if you're in `/home/me/Documents/Cyclone_Robosub/`, then `.` is  `/home/me/Documents/Cyclone_Robosub/` and `..` is `/home/me/Documents/`.
    - You can `cd` to these just like any other directory.
- For more help with terminal commands, check the Internet. Make sure that you specify your OS: the commands can be very different depending on whether you're on a Unix system or a Windows machine.

## Running 

### Using CMake to Build Code
#### Build for a Non-Pi machine (i.e. a regular computer)
1. Open the terminal and `cd` to the project root. If you run `ls` here, you should see folders with names `src/`, `testing/`, etc.
2. Run `cmake -DMOCK_RPI=ON -B build`
3. Run `cd build`
4. Run `make`. You should get one executables: `propulsion_test`.
5. Run with `./propulsion_test`.

#### Build for a Pi
1. Open the terminal and `cd` to the project root. If you run `ls` here, you should see folders with names `src/`, `testing/`, etc.
2. Run `cmake -B build`
3. Run `cd build`
4. Run `make`. You should get one executable: `propulsion_test`.
5. Run with `./propulsion_test`.

### Running Unit Tests
> Before you push code to the repo, you should make sure that you pass all the unit tests. Here's how:
1. Follow the instructions above to build the code (**"Using CMake to Build Code"**)
2. Run with `./propulsion_test`
3. Confirm that all tests pass (are green). If there are any failed (red) tests, check why they're failing and get them fixed! If you get stuck, try using the debugger (see **Troubleshooting**).

### Making Unit Tests
You should write your tests in the `testing/` folder, in the testing file that corresponds with the file or class that you're testing.

The format for a unit test is:
```C++
TEST(Test_Category, Test_Name) {
    //Test here
}
```
`Test_Category` refers to what component you're testing. For example, if you're testing `Command_Interpreter.cpp`, then
the test should be called `CommandInterpreterTest`. `Test_Name` refers to the name of this specific test: for example, if
you are testing that you can execute a command, call it `ExecuteCommand`.

Inside the test, you should add assertions to check that the code is performing as expected. These will be evaluated to determine if the test passes when it is run. These look like `ASSERT_EQ(val1, val2)` (determines whether `val1` is equal to `val1`),`ASSERT_TRUE(val)` (determines whether `val` evaluates to `true` or not), etc. If no assertions are added, the test will pass if it doesn't crash, and fail if it does (without checking if the code is working correctly or not). Other assertions exist as well, and you should reference GTest documentation to learn more about them (https://google.github.io/googletest/reference/assertions.html).

To run these tests, see **"Running Unit Tests"**

## Setting up WSL
WSL stands for Windows Subsystem for Linux. It's developed by Microsoft, and provides you with a Linux terminal environment for your Windows machine. In many ways, this gives you the best of both worlds: the convenience of a Linux filesystem and command interface while still letting you use an operating system that you're familiar with (although you miss out on the clout of being able to say "I'm a Linux user!").

It's not the same as daul-booting or a virtual machine (although those can be great options for gaining access to a Linux environment). WSL doesn't require you to restart your computer every time you want to change which environment your in, and doesn't have the same resource-intensive characteristics of virtual machines. However, you don't get access to a graphical Linux environment, and there are sometimes quirks with WSL (such as shortcut keymapping, among others).

You can find official WSL instructions here: https://learn.microsoft.com/en-us/windows/wsl/install

You can choose any Linux environment that you like, but the easiest option is Ubuntu or an Ubuntu-based distribution. The Raspberry Pis that our code runs on have Ubuntu installed, so running Ubunutu in WSL ensures the best compatibility.

Once it's set up, take some time to get used to the terminal environment. Try moving around to different folders, and explore a bit. The best way to learn is through experience!

## Troubleshooting
- When I try to turn `cmake -B build`, I get an error saying that my version of CMake is too old!
  - The default version installed may be too old for this project. Try removing it (if in a Linux environment, this might be `sudo apt remove cmake`) and installing it again, making sure that you're installing a version $ \geq 3.28$.
- When I run `cmake -B build`, I get an error saying that it can't find my compiler!
  - Double check that gcc/g++ and make are installed and can be run through the terminal.
- When I try to run git (i.e. `git pull`) I get an error saying "not a git repository"!
  - Check that you're in a directory inside the project. This can be confirmed through the `ls` command: make sure that the files displayed match those of the project. 
- I installed WSL, and I can't find my project files through the WSL command line!
  - You'll almost certainly want to move your files into the Linux filesystem offered by WSL eventually, but until then you can access your Windows filesystem from `/mnt/c` within WSL. Try running `cd /mnt/c` and then `ls`: this should display a familiar-looking filesystem that you can navigate to find your project directory.
- Help! My code is crashing / giving me a `SIGSEV` error / not passing the unit tests!
  - Try using the debugger to find where and why your code is breaking! Your IDE probably offers built-in debugging tools. Put a breakpoint where you'd like the code to stop (such as in a function call that you think is crashing), run the
  code in debug mode, and then step through line by line from your breakpoint. Keep an eye on the values of the variables: when one stops matching what you expect, or the program crashes, or both, see if you can work out why the code doesn't match your expectations.

---

# User Resources

## Overview
Command_Interpreter is designed to run on a Raspberry Pi 5 (or 4). It is used to get commands from a main executive and send them to a Raspberry Pi Pico, which will set PWM values to control thruster speed and direction. This code won't run (outside of a testing build) unless it has a Raspberry Pi Pico attached via USB.

## Necessary Setup
To run this code, you will need to update the ID of the Rasberry Pi Pico in the `Serial.cpp` and `Wiring.cpp` files to the corresponding name (found in `\dev\serial\by-id\`). The Pico should be running the code from the MicroPython Pool Testing repo (https://github.com/Cyclone-Robosub/micro-python-pool-test/).

## Command_Intepreter.*
These and `Command.hpp` are the only files that contains code that you should have to actively interact with. Functions should be heavily documented, so it is encouraged to hover over function names to see what parameters represent and how functions should be used.

A Command Interpreter object needs to be given thruster pins and digital pins (although these can be empty if a certain pin type is unused). Thruster pins are those that are to be used to signal to the robot thrusters, should be Hardware PWM pins. Additionally, it needs to be a given three output streams: output, outLog, and errorLog. Output is where standard messages should be sent (you probably want this to be std::cout so that messages are sent to stdout), outLog is where you want standard logging messages to be sent, and errorLog is where you want error messages to be logged. These can be set to the same file if you want everything consolidated, or to `\dev\null` if you want them sent into the abyss.

Once a Command Interpreter is created, with the appropriate pins designated for thrusters and digital pins, execute commands can be sent through the execute functions. These commands will be relayed to the Pi Pico, which will set the corresponding pins to the specified PWM values.

## Command.hpp
This specifies the components of a command to be passed to the Command Interpreter. There are three componenents: acceleration, steady-state, and deceleration. The idea is that the command will bring the robot up to a certain velocity, then maintain that velocity for a certain amount of time, then decelerate back to stopped. PWMs and durations can be specified per each component. If the component is unnecessary (i.e. only a steady-state component is desired), then the other components should be set to a duration of $0$ and the PWMs set to the same values as the used component.

## Wiring.*
This contains code used internally by Command Interpreter to send commands over serial to the Pico. You shouldn't have to interface with this when using Command_Interpreter elsewhere.

---

Code by Propulsion subteam of UC Davis Cyclone Robosub. README by William Barber.
