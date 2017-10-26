function v = CR_EULER_FREE_POS_X()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 22);
  end
  v = vInitialized;
end
