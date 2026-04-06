export var AbsolutePoseFlags;
(function (AbsolutePoseFlags) {
    AbsolutePoseFlags[AbsolutePoseFlags["RTK"] = 1] = "RTK";
    AbsolutePoseFlags[AbsolutePoseFlags["FIXED"] = 2] = "FIXED";
    AbsolutePoseFlags[AbsolutePoseFlags["FLOAT"] = 4] = "FLOAT";
    AbsolutePoseFlags[AbsolutePoseFlags["DEAD_RECKONING"] = 8] = "DEAD_RECKONING";
})(AbsolutePoseFlags || (AbsolutePoseFlags = {}));
