#!/bin/bash

#
# 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#
# script for automated snapshot builds for GTSAM using
# git-build-package (gbp)


usage() {
    echo "script to create gtsam snapshots using git buildpackage (gbp)"
    echo
    echo "mandatory options:"
    echo
    echo " -b <snapshot base version>. This determines the pristine tar ball"
    echo "    to use for reference. Example: 4.1.0"
    echo " -k <fingerprint>. Fingerprint of the secret GPG key to use."
    echo "    Key must be registered with your ubuntu PPA and with github)"
    echo " -u <url of upstream repo>. The repo from which to pull the nightly"
    echo "    development version, e.g. https://github.com/borglab/gtsam.git"
    echo " -v <vendor>. Currently only ubuntu is supported as vendor"
    echo " -p <url of ppa>. Url of ppa, e.g. ppa:myusername/myppa"
    echo " -f <flavor>. Which library flavor to build (gtsam, gtsam-no-tbb, ...)"
    echo " -n <username>. Username, e.g. \"My Name\" (in quotes!). This name"
    echo "    will be used in commit messages"
    echo " -d <debian_branch>. debian packaging branch used will be: $vendor/$debian_branch"
    echo " -e <email>. Email, e.g. user@foo.bar. Must be your github registered email"
}

OPTIND=1
while getopts "h?v:p:b:u:k:n:d:e:f:" opt; do
    case "$opt" in
	h|\?)
	    usage
	    exit 0
	    ;;
        v) vendor=$OPTARG
	   ;;
	p) ppa=$OPTARG
	   ;;
        b) snapshot_base=$OPTARG
	   ;;
	u) upstream=$OPTARG
	   ;;
	n) user_name=$OPTARG
	   ;;
	d) debian_branch=$OPTARG
	   ;;
	e) email=$OPTARG
	   ;;
	f) flavor=$OPTARG
	   ;;
	k) gpg_key=$OPTARG
	   ;;
    esac
done

shift $((OPTIND-1))

[ "${1:-}" = "--" ] && shift

if [ -z ${vendor+x} ] || [ -z ${ppa+x} ] || [ -z ${snapshot_base+x} ] || [ -z ${upstream+x} ] || [ -z ${gpg_key+x} ] || [ -z ${user_name+x} ] || [ -z ${email+x} ] || [ -z ${flavor+x} ] || [ -z ${debian_branch+x} ] ; then
    echo " ERROR: missing command line parameter. All parameters must be present!"
    echo
    usage
    exit 1
fi

packaging_branch=${vendor}/${debian_branch}

echo "--------------------------------------"
echo "snapshot base version: $snapshot_base"
echo "flavor:                $flavor"
echo "packaging branch:      $packaging_branch"
echo "upstream repo:         $upstream"
echo "upload ppa:            $ppa"
echo "using key:             $gpg_key"
echo "user name              $user_name"
echo "email                  $email"
echo
echo 'current working directory:' `pwd`
echo "----- gpg secret keys ---------"
gpg --list-secret-keys
echo '----- directory: --------'
pwd
ls -la

# set upstream (where the developers commit to)
git remote add upstream $upstream
# fetch the upstream development branch
git fetch upstream develop
echo 'branches after upstream fetch:'
git branch -r
echo 'config after upstream fetch:'
git config -l

# get the latest hash so we can check if we already processed this one
git_hash=`git log -1 --pretty=format:"%H" upstream/develop`

# this is the user under which the commits will be reported
git config --global user.email $email
git config --global user.name  "$user_name"

# grab the snapshot branch from the packaging repo.
# It will get us to where we left off, i.e. when the last snapshot was run
echo 'branches before config checkout'
git branch -r
echo 'git config before checkout out'
git config -l
echo 'checking out last snapshot'
git checkout ${packaging_branch}  # switch to snapshot tracking branch

hash_store_file=debian/git_last_snapshot_hash.txt
if grep -q $git_hash "$hash_store_file"; then
    echo "snapshot of $git_hash was already taken, aborting!"
fi

# retrieve the pristine tarball from the pristine-tar branch
#git fetch packaging pristine-tar:pristine-tar
git fetch origin pristine-tar:pristine-tar
echo 'branch after pristine-tar fetch:'
git branch -r

gbp export-orig --pristine-tar

# merge in the changes new in the develop branch
# (for testing purposes, you can here checkout an older version of
#  the development tree, and work your way forward by several commits
# git merge 915702116f7316bdb6f9b0a512d26eea3d49600b -m "merge test commit")
git merge upstream/develop -m "merge develop branch"

# update changelog to capture all the changes between now and last time
# The 'snapshot' feature automatically increments the version number
# from the changelog file. It also sets the distribution to UNRELEASED,
# no matter what you pass in as "distribution".
gbp dch --debian-branch=${packaging_branch} --distribution=UNRELEASED --snapshot --git-author

# get the new snapshot version from changelog
snap=`head -1 debian/changelog | sed 's/.*[(]//g; s/[)].*//g'`

# remove all previous patches so they don't collide with the new ones,
# which are computed between the version stored in the pristine tar
# ball and the current snapshot.
git rm -r debian/patches/*
# now build patch file for difference between pristine -> snapshot
# (must disable editor to get away without commit message)
EDITOR=/bin/true dpkg-source --include-removal --commit . ${snap}.patch
rm -rf .pc # remove this automatically created directory to avoid error

# commit updated changelog, patch, and binary files
git add debian/changelog debian/patches debian/source/include-binaries
git commit -a -m "updated changelog and patch files for snapshot $snap"

#
# loop over distros so we can bump the version number for each one
#
for distro in focal jammy noble
do
    # remove any old build files
    rm -f ../${flavor}_*.dsc ../${flavor}_*.build ../${flavor}_*.buildinfo ../${flavor}_*.changes ../*.upload

    # in-place replace of UNRELEASED; with specific distro in changelog file.
    # the -z option only replaces the first occurence
    sed -i -z "s/UNRELEASED\;/${distro}\;/g" debian/changelog
    # must commit changes to the changelog file or else the buildpackage
    # will barf
    git add debian/changelog
    git commit -m "modified changelog for distro $distro"
    # this will actually build the source package, i.e. create the
    # stuff that can be uploaded to ubuntu's ppa farm for building
    gbp buildpackage -k${gpg_key} -S -sa --git-debian-branch=${packaging_branch} --source-option="--include-removal"

    # upload to ubuntu ppa server for building
    pushd ..
    dput "$ppa" ${flavor}_*_source.changes
    popd
    
    # now update the changelog to capture the new commits. This will
    # also bump the snapshot version for the next distro, which
    # needs to happen because the ubuntu build server will reject
    # anything that does not have an increasing version number over the
    # previously uploaded source package
    gbp dch --debian-branch=${packaging_branch} --distribution=UNRELEASED --snapshot --git-author

    # update snapshot version, which will be used in the next iteration
    snap=`head -1 debian/changelog | sed 's/.*[(]//g; s/[)].*//g'`

    # sleep for some time such that the previous ppa upload finishes
    # otherwise the current one may overtake the previous one, and it
    # gets rejected because of non-increasing sequence numbers
    echo 'sleeping for 1min ....'
    sleep 60
done

# keep the git hash of the point where we just took the snapshot
# (comment out this line for testing!)
echo $git_hash > $hash_store_file
echo "check for file $hash_store_file"
ls -la $hash_store_file
# commit the final snapshot number to the changelog so we have it
# next time we run this script
git add debian/changelog
git add $hash_store_file
git commit -m "commit of final snapshot version number update"

# finally, push the changes to the snapshot branch back home
git branch -r
git push origin ${packaging_branch}
