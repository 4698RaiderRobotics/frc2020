# Documentation Header - change me please.

# Install Git 

Here is the authority: https://git-scm.com/downloads

You will need to create a github account.  Go to github.com and sign up for educational account.  If you need to be a collaborator

There are lots of great guides out there. Don't be afraid to explore.


# Get the repository.

Open your terminal ( on windows it's ?).
Change directory to the folder you want the repo to be stored in.  If you just want home directory then:

```
# this is a comment and won't be run because it starts with a #.
# you can add comments to code blocks that are run in the terminal.
# ...
# change directories to home ( or what ever location you want to store the repo in)

# ~ means your home directory ( where photos, documents, desktop are stored).
cd ~
git clone git@github.com:RaiderRobotics4698/frc2020.git

# the code should now be clone into ~/frc2020.

cd ~/frc2020
ls -l
# you should have the contents of the repo listed.
# you can setup your visual studio environment now.
```

# Raiders Git Quick Commands.
Primarily we will be working with five git commands.  We'll go over branch and merge later.

## git pull ( Pull other peoples changes into my code )
You should git pull on regular basis.  We'll go into more detail.
Whenever you sit down to code, start with a git pull.

```
cd ~/frc2020
git pull
## git status (What has changed on my local system)
```
## git status
You will get a list of files and directories that have changed.  Read the messages printed; it's actually quite smart.
```
git status
```

## git add  ( Add my code changes to a commit unit)

The status command will tell you what you need to commit.  You can be lazy and just commit everything, or invidual files. 
```
#lazy ( or concise) just go for it all.
git add .

#a just a file ( probably copied from git status ).  this is just a an example, don't copy and paste.
	git add src/coolgadget.h src/coolgadget.cpp
```	

## git commit ( commit my code changes into the record)
After you have done your "git add" for your changes it time to record that code in the record.
```
git commit -m 'write a message for posterity ( or at least something that describe what your committing).  This is a record of what you have accomplished'
```

Now read the messages printed out on the screen.  Again it's very informative.

## git push ( push my commit for the world to see)

After you've done your "git commit" command it's time to push to the world.

```
git push
```

Read the message and make sure there isn't any errors. 

## git branch ( I don't want to break everything so I'll make my own safe space to code)

We'll go through branching shortly once we have our base environments set up and compiling with the standard FRC libraries.

In a nutshell this is how we protect the Robot and each other from ourselves.  It's our safe space to create and break things.

